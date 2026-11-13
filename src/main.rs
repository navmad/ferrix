#![no_std]
#![no_main]

pub mod boards;
pub mod companion;
pub mod display;
pub mod fault;
pub mod gps;
pub mod input;
pub mod meshcore;
pub mod radio;
pub mod storage;
pub mod telemetry;

use defmt::info;
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::peripherals;
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::usb;
use embassy_time::Timer;
#[cfg(feature = "thinknode-m1")]
use static_cell::StaticCell;

use crate::boards::{Board, CurrentBoard};
use crate::companion::protocol::CompanionNode;
use crate::companion::usb::{UsbCommand, USB_COMMAND_CHANNEL};
#[cfg(feature = "thinknode-m1")]
use crate::display::{EpdFrameBuffer, NodeDisplay, Ssd1681, ThinkNodeDisplay};
use crate::input::{InputDevice, InputEvent};
use crate::meshcore::{InboundAction, LocalIdentity, MeshEngine, Packet};
use crate::radio::{lora_phy_task, LoraConfig, LoraPhyChannelRadio, LoraRadio};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    TWISPI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
});

static VBUS_DETECT: StaticCell<embassy_nrf::usb::vbus_detect::SoftwareVbusDetect> = StaticCell::new();

#[derive(Clone, Copy, Debug)]
pub struct PendingMessageAck {
    pub crc: u32,
    pub send_instant_ms: u64,
}

static PENDING_MESSAGE_ACKS: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
    core::cell::RefCell<heapless::Vec<PendingMessageAck, 8>>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(heapless::Vec::new()));

fn track_pending_ack(crc: u32) {
    let now = embassy_time::Instant::now().as_millis();
    PENDING_MESSAGE_ACKS.lock(|a| {
        let mut list = a.borrow_mut();
        if list.is_full() {
            list.remove(0);
        }
        let _ = list.push(PendingMessageAck {
            crc,
            send_instant_ms: now,
        });
    });
}

fn handle_direct_message_confirmed(crc: u32) {
    let now = embassy_time::Instant::now().as_millis();
    let rtt_ms = PENDING_MESSAGE_ACKS.lock(|a| {
        let mut list = a.borrow_mut();
        list.iter().position(|p| p.crc == crc).map(|pos| {
            let item = list.remove(pos);
            now.saturating_sub(item.send_instant_ms) as u32
        })
    });

    // ACK snooping can see ACKs for other nodes; only confirm a message we
    // actually sent.
    let Some(rtt_ms) = rtt_ms else {
        return;
    };

    info!("MeshCore Direct Message Delivery ACK Confirmed! (crc {:08x}, RTT: {} ms)", crc, rtt_ms);

    // Push PUSH_CODE_SEND_CONFIRMED (0x82) to BLE companion app
    let mut push_frame = heapless::Vec::<u8, 256>::new();
    let _ = push_frame.push(crate::companion::protocol::PUSH_CODE_SEND_CONFIRMED);
    let _ = push_frame.extend_from_slice(&crc.to_le_bytes());
    let _ = push_frame.extend_from_slice(&rtt_ms.to_le_bytes());
    let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(push_frame);
}

// Static allocations for display framebuffer
#[cfg(feature = "thinknode-m1")]
static EPD_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();

/// Number of page turns between full screen refreshes (clears e-ink ghosting)
const EPD_FULL_REFRESH_INTERVAL: u32 = 9;

// ---------------------------------------------------------------------------
// Fatal-error indicators
//
// The red LED (P1.04) on the ThinkNode M1 is driven by the charger IC, so it
// cannot be used as a firmware status. Use the MCU-controlled blue LED (P0.13)
// instead. Patterns (all on blue):
//   - normal boot:  3 fast blinks, then the slow main-loop heartbeat
//   - panic:        3 quick blinks, repeated forever
//   - HardFault:    6 quick blinks, repeated forever
//   - SoftDevice:  10 quick blinks, repeated forever
// A dark blue LED therefore means the CPU is stuck in a loop that never
// reaches any handler (a hang), while a blinking pattern identifies the fault.
// ---------------------------------------------------------------------------

const P0_BASE: u32 = 0x5000_0000;
const P0_PIN_CNF13: *mut u32 = (P0_BASE + 0x504 + 4 * 13) as *mut u32;
const P0_OUTSET: *mut u32 = (P0_BASE + 0x508) as *mut u32;
const P0_OUTCLR: *mut u32 = (P0_BASE + 0x50C) as *mut u32;
const P0_DIRSET: *mut u32 = (P0_BASE + 0x518) as *mut u32;
const BLUE_LED: u32 = 1 << 13;

/// ~35 ms at 64 MHz. Used instead of a timer so it still works when the
/// scheduler, the SoftDevice or interrupts are wedged.
#[inline(never)]
fn fatal_delay() {
    cortex_m::asm::delay(2_000_000);
}

fn blue_on() {
    unsafe { P0_OUTSET.write_volatile(BLUE_LED) }
}

fn blue_off() {
    unsafe { P0_OUTCLR.write_volatile(BLUE_LED) }
}

fn blue_init() {
    unsafe {
        // Dir=output, input buffer disconnected.
        P0_PIN_CNF13.write_volatile(0x0000_0003);
        P0_DIRSET.write_volatile(BLUE_LED);
    }
}

/// Short, unmistakable burst used to classify the failure without a debugger.
fn fatal_blink_burst(count: u32) {
    blue_init();
    for _ in 0..count {
        blue_on();
        fatal_delay();
        blue_off();
        fatal_delay();
    }
    // Gap between bursts.
    for _ in 0..4 {
        fatal_delay();
    }
}

/// Repeat a burst forever. Used for fatal faults that cannot recover.
pub(crate) fn fatal_blink_forever(count: u32) -> ! {
    loop {
        fatal_blink_burst(count);
    }
}

/// Blink a burst so the failure is visible, then reset the core. The panic
/// record lives in `.uninit` RAM, which survives a soft reset, so the next
/// boot can report it over USB (`get panic`).
fn fatal_finish(burst: u32) -> ! {
    fatal_blink_burst(burst);
    if crate::fault::reset_budget_exhausted() {
        // Panicking after several panic-triggered resets: stop rebooting so the
        // failure can at least be observed.
        fatal_blink_forever(burst)
    } else {
        crate::fault::reset_now()
    }
}

/// defmt's `panic!`/`unwrap!`/`assert!` macros log a message and then call
/// `_defmt_panic`. Override the default hook so we can capture the return
/// address, which points directly at the failing expression.
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    let lr: u32;
    // SAFETY: reading LR has no side effects.
    unsafe {
        core::arch::asm!("mov {}, lr", out(reg) lr);
    }
    crate::fault::record_defmt_panic(lr);
    fatal_finish(3)
}

/// Custom panic handler.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // No logging here: defmt can block if a debugger configured blocking mode,
    // which would prevent the diagnostic reset.
    crate::fault::record_panic(info);
    fatal_finish(3)
}

/// HardFault handler.
#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    crate::fault::record_hardfault(frame.pc(), frame.lr());
    fatal_finish(6)
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    use embassy_nrf::interrupt::{InterruptExt, Priority};

    // SoftDevice S140 reserves interrupt priorities P0, P1, and P4.
    // Configure all Embassy system and peripheral interrupts to P2 or P3.
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.gpiote_interrupt_priority = Priority::P2;
    nrf_config.time_interrupt_priority = Priority::P2;
    let p = embassy_nrf::init(nrf_config);

    embassy_nrf::interrupt::USBD.set_priority(Priority::P3);
    embassy_nrf::interrupt::TWISPI0.set_priority(Priority::P3);
    embassy_nrf::interrupt::TWISPI1.set_priority(Priority::P3);

    let mut board = CurrentBoard::init(p);

    // A panic/hardfault in the previous boot leaves a record in `.uninit` RAM
    // and resets the core. Report it (and keep it for the `get panic` CLI).
    if let Some(panic) = crate::fault::peek() {
        defmt::error!(
            "RECOVERED PANIC: {} at {}:{}:{} pc={=u32:#010x}",
            panic.msg.as_str(),
            panic.file.as_str(),
            panic.line,
            panic.column,
            panic.pc
        );
    }

    // Initialize persistent configuration from flash
    let (node_name, custom_key) = crate::storage::init_config();

    // Initialize native MeshCore Routing Engine & Local Cryptographic Identity.
    // The stored key is MeshCore's 64-byte expanded form; old configs are
    // migrated from their 32-byte seed by storage::init_config().
    let local_id = if let Some(key) = custom_key {
        LocalIdentity::from_private(node_name.as_str(), &key).unwrap_or_else(|| {
            info!("Stored private key is reserved or invalid; using hardware seed");
            LocalIdentity::from_seed(node_name.as_str(), &crate::boards::hardware_seed())
        })
    } else {
        LocalIdentity::from_seed(node_name.as_str(), &crate::boards::hardware_seed())
    };
    let pub_key = local_id.pub_key_bytes();
    let priv_key = local_id.private_key_bytes();
    let mut mesh = MeshEngine::new(local_id, false);

    // 0. Initialize USB CDC-ACM Serial & 1200-baud DFU Auto-Reset FIRST
    // This guarantees serial CLI, packet logging, and DFU auto-reset are always available.
    let vbus: &'static embassy_nrf::usb::vbus_detect::SoftwareVbusDetect =
        &*VBUS_DETECT.init(embassy_nrf::usb::vbus_detect::SoftwareVbusDetect::new(true, true));
    let usb_driver = usb::Driver::new(board.usbd, Irqs, vbus);
    let (usb_dev, cdc_class) = crate::companion::usb::init_usb(usb_driver);
    spawner.spawn(crate::companion::usb::usb_task(usb_dev)).unwrap();
    spawner.spawn(crate::companion::usb::cdc_acm_task(cdc_class, pub_key, priv_key)).unwrap();
    spawner.spawn(crate::companion::ble::contact_task()).unwrap();

    // Initial node companion state
    let mut node = CompanionNode::new(
        node_name.as_str(),
        CurrentBoard::NAME,
        CurrentBoard::CAPABILITIES,
        pub_key,
        priv_key,
    );

    // Initialize BLE Companion peripheral (Nordic UART Service)
    crate::companion::ble::init_ble_companion(spawner, pub_key, priv_key);

    info!("====================================================");
    info!(" MeshCore Companion Firmware (ferrix) ");
    info!(" Active Board: {} ", CurrentBoard::NAME);
    info!(" Runtime: Embassy async no_std (nRF52840 Cortex-M4) ");
    info!("====================================================");

    // Visual Boot Confirmation: 3 fast blue blinks indicate successful MCU & runtime start
    for _ in 0..3 {
        board.telemetry.blue_led.set_high(); // ON (active-high)
        Timer::after_millis(70).await;
        board.telemetry.blue_led.set_low(); // OFF
        Timer::after_millis(70).await;
    }

    // 1. Initialize SPI0 for LoRa Radio (8 MHz)
    let mut spim0_config = spim::Config::default();
    spim0_config.frequency = spim::Frequency::M8;
    spim0_config.mode = spim::MODE_0;

    let spim0 = Spim::new(
        board.spim0,
        Irqs,
        board.lora.sck,
        board.lora.miso,
        board.lora.mosi,
        spim0_config,
    );

    // Spawn background lora-phy radio task on ThinkNode M1 SX1262
    spawner.spawn(lora_phy_task(
        spim0,
        board.lora.cs,
        board.lora.reset,
        board.lora.busy,
        board.lora.dio1,
        board.lora.rf_sw,
    )).unwrap();

    let mut lora = LoraPhyChannelRadio::new();
    let _ = lora.init(&LoraConfig::default()).await;
    lora.start_receive().await;

    // 2. Initialize Display via generic NodeDisplay trait
    #[cfg(feature = "thinknode-m1")]
    let mut display = if CurrentBoard::HAS_DISPLAY {
        if let Some(disp) = board.display {
            let mut spim1_config = spim::Config::default();
            spim1_config.frequency = spim::Frequency::M4;
            spim1_config.mode = spim::MODE_0;

            let spim1 = Spim::new_txonly(
                board.spim1,
                Irqs,
                disp.sclk,
                disp.mosi,
                spim1_config,
            );

            let epd = Ssd1681::new(
                spim1,
                disp.cs,
                disp.dc,
                disp.reset,
                disp.busy,
                disp.backlight,
            );

            let fb = EPD_FRAMEBUFFER.init(EpdFrameBuffer::new());
            let mut d = ThinkNodeDisplay::new(epd, fb);

            info!("Initializing display and drawing MeshCore Dashboard (Full calibration refresh)...");
            d.init().await.ok();
            d.render_dashboard_full(&node).await.ok();
            info!("Dashboard rendered successfully!");
            Some(d)
        } else {
            None
        }
    } else {
        None
    };

    // Ensure blue LED is OFF after boot (active-high: Low = OFF)
    board.telemetry.blue_led.set_low();

    let mut rx_buf = [0u8; 256];
    let mut tick = 0u32;
    let mut page_turns = 0u32;

    loop {
        // Poll for incoming LoRa packets
        if let Some((len, rssi, snr)) = lora.poll_receive(&mut rx_buf).await {
            info!("Received LoRa packet: {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
            board.telemetry.blue_led.set_high(); // ON
            node.rx_packets = node.rx_packets.saturating_add(1);
            node.last_rssi = rssi;
            node.last_snr = snr;

            // Stream raw packet to companion app via PUSH_CODE_LOG_RX_DATA (0x88)
            // Layout: [0x88, (snr * 4) as i8, rssi as i8, raw_lora_packet_bytes...]
            let mut log_rx_push = heapless::Vec::<u8, 256>::new();
            let _ = log_rx_push.push(crate::companion::protocol::PUSH_CODE_LOG_RX_DATA);
            let _ = log_rx_push.push(snr.saturating_mul(4) as u8);
            let _ = log_rx_push.push(rssi as i8 as u8);
            let copy_lora_len = len.min(256 - 3);
            let _ = log_rx_push.extend_from_slice(&rx_buf[..copy_lora_len]);
            let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(log_rx_push);

            // Synchronize real radio telemetry to shared USB CLI status
            crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.rx_packets = tel.rx_packets.saturating_add(1);
                tel.last_rssi = rssi;
                tel.last_snr = snr;
                tel.has_rx = true;
            });

            // Deserialize and dispatch through MeshCore Network Engine
            if let Ok(pkt) = Packet::read_from(&rx_buf[..len]) {
                match mesh.handle_inbound_packet(pkt) {
                    InboundAction::ReceivedAdvert(adv) => {
                        info!("MeshCore Advert from: {}", adv.name.as_str());
                        let node_id = [adv.pub_key[0], adv.pub_key[1], adv.pub_key[2], adv.pub_key[3]];

                        // Check if this contact already exists in our list
                        let contact_type = CompanionNode::get_contact_type_from_flags(adv.flags);
                        let should_add = node.should_auto_add_contact_type(contact_type);

                        crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                            let mut list = c.borrow_mut();
                            if let Some(existing) = list.iter_mut().find(|ct| ct.pub_key == adv.pub_key) {
                                // Anti-replay: ignore an advert no newer than the last accepted one.
                                if existing.last_advert_ts != 0 && adv.timestamp <= existing.last_advert_ts {
                                    return;
                                }
                                existing.callsign = adv.name.clone();
                                existing.last_seen_epoch = node.epoch_time as u32;
                                existing.last_advert_ts = adv.timestamp;
                                existing.rssi = rssi;
                                existing.snr = snr;
                                existing.latitude = adv.latitude;
                                existing.longitude = adv.longitude;
                                existing.flags = adv.flags;
                            } else {
                                // New contact - apply auto-add rules
                                if !should_add {
                                    // Contact type not enabled for auto-add, skip
                                    return;
                                }

                                // Try to add the contact
                                if list.len() < list.capacity() {
                                    // Space available
                                    let mut cs = heapless::String::new();
                                    let _ = cs.push_str(adv.name.as_str());
                                    let _ = list.push(crate::meshcore::mesh::DiscoveredContact {
                                        pub_key: adv.pub_key,
                                        callsign: cs,
                                        node_id,
                                        last_seen_epoch: node.epoch_time as u32,
                                        rssi,
                                        snr,
                                        latitude: adv.latitude,
                                        longitude: adv.longitude,
                                        flags: adv.flags,
                                        out_path: [0u8; 64],
                                        out_path_len: 0xFF,
                                        last_advert_ts: adv.timestamp,
                                    });
                                } else if node.should_overwrite_when_full() {
                                    // List is full, find oldest non-favorite contact to overwrite
                                    let mut oldest_idx = None;
                                    let mut oldest_lastmod = u32::MAX;
                                    for (idx, ct) in list.iter().enumerate() {
                                        let is_favorite = (ct.flags & crate::companion::protocol::CONTACT_FLAG_FAVORITE) != 0;
                                        if !is_favorite && ct.last_seen_epoch < oldest_lastmod {
                                            oldest_lastmod = ct.last_seen_epoch;
                                            oldest_idx = Some(idx);
                                        }
                                    }
                                    if let Some(idx) = oldest_idx {
                                        // Send push notification that contact is being overwritten
                                        let overwritten_key = list[idx].pub_key;
                                        let mut push = heapless::Vec::<u8, 256>::new();
                                        let _ = push.push(crate::companion::protocol::PUSH_CODE_CONTACT_DELETED);
                                        let _ = push.extend_from_slice(&overwritten_key);
                                        let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(push);

                                        // Replace with new contact
                                        let mut cs = heapless::String::new();
                                        let _ = cs.push_str(adv.name.as_str());
                                        list[idx] = crate::meshcore::mesh::DiscoveredContact {
                                            pub_key: adv.pub_key,
                                            callsign: cs,
                                            node_id,
                                            last_seen_epoch: node.epoch_time as u32,
                                            rssi,
                                            snr,
                                            latitude: adv.latitude,
                                            longitude: adv.longitude,
                                            flags: adv.flags,
                                            out_path: [0u8; 64],
                                            out_path_len: 0xFF,
                                            last_advert_ts: adv.timestamp,
                                        };
                                    } else {
                                        // All contacts are favorites, can't overwrite
                                        // Send push notification that contacts list is full
                                        let mut push = heapless::Vec::<u8, 256>::new();
                                        let _ = push.push(crate::companion::protocol::PUSH_CODE_CONTACTS_FULL);
                                        let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(push);
                                    }
                                } else {
                                    // List is full and overwrite is disabled
                                    // Send push notification that contacts list is full
                                    let mut push = heapless::Vec::<u8, 256>::new();
                                    let _ = push.push(crate::companion::protocol::PUSH_CODE_CONTACTS_FULL);
                                    let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(push);
                                }
                            }
                        });
                    }
                    InboundAction::ReceivedGroupText { channel_idx, path_len, packet: grp } => {
                        info!("MeshCore Group Text [ch {}]: <{}> {}", channel_idx, grp.sender.as_str(), grp.message.as_str());
                        node.unread_messages = node.unread_messages.saturating_add(1);
                        crate::companion::usb::MESSAGE_LOG.lock(|m| {
                            let mut list = m.borrow_mut();
                            if list.is_full() {
                                list.remove(0);
                            }
                            let mut s_sender = heapless::String::new();
                            let _ = s_sender.push_str(grp.sender.as_str());
                            let mut s_text = heapless::String::new();
                            let _ = s_text.push_str(grp.message.as_str());
                            let _ = list.push(crate::meshcore::mesh::ReceivedMessage {
                                channel_idx,
                                path_len,
                                sender: s_sender,
                                sender_pub: [0u8; 32],
                                text: s_text,
                                timestamp: grp.timestamp,
                                rssi,
                                snr,
                                is_direct: false,
                            });
                        });

                        // Prompt companion app with PUSH_CODE_MSG_WAITING (0x83)
                        let mut msg_wait = heapless::Vec::<u8, 256>::new();
                        let _ = msg_wait.push(crate::companion::protocol::PUSH_CODE_MSG_WAITING);
                        let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(msg_wait);
                    }
                    InboundAction::ReceivedDirectText { msg, sender_pub } => {
                        info!(
                            "MeshCore Direct Msg from {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}: {}",
                            sender_pub[0], sender_pub[1], sender_pub[2],
                            sender_pub[3], sender_pub[4], sender_pub[5],
                            msg.text.as_str()
                        );
                        node.unread_messages = node.unread_messages.saturating_add(1);
                        crate::companion::usb::MESSAGE_LOG.lock(|m| {
                            let mut list = m.borrow_mut();
                            if list.is_full() {
                                list.remove(0);
                            }
                            let mut s_sender = heapless::String::new();
                            use core::fmt::Write;
                            let _ = write!(
                                s_sender,
                                "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                                sender_pub[0],
                                sender_pub[1],
                                sender_pub[2],
                                sender_pub[3],
                                sender_pub[4],
                                sender_pub[5]
                            );
                            let mut s_text = heapless::String::new();
                            let _ = s_text.push_str(msg.text.as_str());
                            let _ = list.push(crate::meshcore::mesh::ReceivedMessage {
                                channel_idx: 0,
                                path_len: 0,
                                sender: s_sender,
                                sender_pub,
                                text: s_text,
                                timestamp: msg.timestamp,
                                rssi,
                                snr,
                                is_direct: true,
                            });
                        });

                        // Prompt companion app with PUSH_CODE_MSG_WAITING (0x83)
                        let mut msg_wait = heapless::Vec::<u8, 256>::new();
                        let _ = msg_wait.push(crate::companion::protocol::PUSH_CODE_MSG_WAITING);
                        let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(msg_wait);
                    }
                    InboundAction::ReceivedRepeaterEcho { pkt_hash, hops } => {
                        info!("MeshCore Message echoed by repeater! ({} hops, pkt_hash: {:?})", hops, pkt_hash);
                    }
                    InboundAction::ReceivedAck(ack) => {
                        info!("MeshCore ACK received (crc {:08x})", ack.crc);
                        handle_direct_message_confirmed(ack.crc);
                    }
                    InboundAction::ReceivedPath { packet: path, sender_pub } => {
                        info!(
                            "MeshCore PATH from {:02x}{:02x}: {} hops, extra {}",
                            sender_pub[0], sender_pub[1], path.hop_count(), path.extra_type
                        );
                        crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                            if let Some(ct) =
                                c.borrow_mut().iter_mut().find(|ct| ct.pub_key == sender_pub)
                            {
                                ct.out_path = path.path;
                                ct.out_path_len = path.path_len;
                            }
                        });
                        // A PATH return can carry an ACK as its extra.
                        if path.extra_type & 0x0F == 3 && path.extra.len() >= 4 {
                            if let Ok(ack) =
                                crate::meshcore::packets::AckPacket::decode(path.extra.as_slice())
                            {
                                handle_direct_message_confirmed(ack.crc);
                            }
                        }
                    }
                    InboundAction::DropDuplicate => {
                        info!("MeshCore: dropped duplicate packet");
                    }
                    InboundAction::DropHopLimitExceeded => {
                        info!("MeshCore: dropped packet exceeding hop limit");
                    }
                    _ => {}
                }
            } else {
                info!("Received non-MeshCore or legacy beacon frame");
            }

            Timer::after_millis(50).await;
            board.telemetry.blue_led.set_low(); // OFF
        }

        // Check for commands from USB CDC-ACM CLI (e.g. set name, time sync, manual advert trigger)
        if let Ok(cmd) = USB_COMMAND_CHANNEL.try_receive() {
            match cmd {
                UsbCommand::SetName(new_name) => {
                    info!("Updating node callsign to: {}", new_name.as_str());
                    mesh.set_callsign(new_name.as_str());
                    node.callsign = new_name;
                    #[cfg(feature = "thinknode-m1")]
                    if let Some(d) = &mut display {
                        d.render_dashboard(&node).await.ok();
                    }
                }
                UsbCommand::SetPrivateKey(priv_key) => {
                    info!("Updating node private & public keys");
                    if mesh.set_private_key(&priv_key) {
                        node.private_key = priv_key;
                        node.public_key = *mesh.identity.pub_key();
                    } else {
                        info!("Rejected reserved private key");
                    }
                }
                UsbCommand::SetTime(epoch) => {
                    info!("Node epoch time updated to: {}", epoch);
                    node.epoch_time = epoch;
                    crate::companion::usb::set_device_time(epoch as u32);
                }
                UsbCommand::TriggerAdvert { flood } => {
                    info!("Queuing MeshCore Advert (flood: {})", flood);
                    node.tx_beacons = node.tx_beacons.saturating_add(1);
                    let time = crate::companion::usb::get_device_time();
                    let _ = mesh.queue_advert(
                        time,
                        node.latitude,
                        node.longitude,
                        flood,
                    );
                }
                UsbCommand::SetAdvertLatLon { lat, lon } => {
                    node.latitude = lat as f32 / 1_000_000.0;
                    node.longitude = lon as f32 / 1_000_000.0;
                }
                UsbCommand::SetAutoAddConfig { config, max_hops } => {
                    node.autoadd_config = config;
                    node.autoadd_max_hops = max_hops;
                }
                UsbCommand::SendGroupText { channel_idx, timestamp, text } => {
                    info!("Queuing MeshCore Group Text on ch {}: {} (ts: {})", channel_idx, text.as_str(), timestamp);
                    // Copy the channel secret out of the lock before computing channel_hash.
                    // Running SHA-256 inside the lock callback can block the BLE SoftDevice.
                    let secret = crate::companion::usb::CHANNELS.lock(|chs| {
                        let list = chs.borrow();
                        if (channel_idx as usize) < list.len() {
                            list[channel_idx as usize].secret
                        } else {
                            [0u8; 16]
                        }
                    });
                    let ch_hash = crate::meshcore::crypto::channel_hash(&secret);
                    node.tx_beacons = node.tx_beacons.saturating_add(1);
                    let _ = mesh.queue_group_text(ch_hash, &secret, text.as_str(), timestamp);
                }
                UsbCommand::SendDirectText {
                    dest_prefix,
                    timestamp,
                    txt_type,
                    attempt,
                    text,
                } => {
                    info!(
                        "Queuing MeshCore Direct Text to {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}: {}",
                        dest_prefix[0], dest_prefix[1], dest_prefix[2],
                        dest_prefix[3], dest_prefix[4], dest_prefix[5],
                        text.as_str()
                    );
                    node.tx_beacons = node.tx_beacons.saturating_add(1);
                    let peer = crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| &ct.pub_key[..6] == &dest_prefix[..])
                            .map(|ct| (ct.pub_key, ct.out_path, ct.out_path_len))
                    });
                    match peer {
                        Some((dest_pub, out_path, out_path_len)) => {
                            match mesh.queue_direct_text(
                                &dest_pub,
                                text.as_str(),
                                timestamp,
                                txt_type,
                                attempt,
                                out_path_len,
                                &out_path,
                            ) {
                                Ok(crc) => track_pending_ack(crc),
                                Err(_) => info!("Direct message send failed (text or encryption)"),
                            }
                        }
                        None => info!("Direct message send failed: contact not found"),
                    }
                }
                UsbCommand::SendControlData { data } => {
                    if !data.is_empty() {
                        if let Ok(cp) = crate::meshcore::packets::ControlPacket::new(
                            data[0],
                            &data[1..],
                        ) {
                            if let Ok(pkt) = cp.to_packet(
                                crate::meshcore::packet::ROUTE_TYPE_DIRECT,
                            ) {
                                mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                                let _ = mesh.tx_queue.push(pkt);
                            }
                        }
                    }
                }
                UsbCommand::SendRawData {
                    path_len,
                    path,
                    payload,
                } => {
                    if Packet::is_valid_path_len(path_len) {
                        let mut pkt = Packet::new(
                            crate::meshcore::packet::ROUTE_TYPE_DIRECT,
                            crate::meshcore::packet::PAYLOAD_TYPE_RAW_CUSTOM,
                        );
                        if pkt.set_payload(payload.as_slice()).is_ok() {
                            pkt.path_len = path_len;
                            let n = pkt.path_byte_len();
                            pkt.path[..n].copy_from_slice(&path[..n]);
                            mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                            let _ = mesh.tx_queue.push(pkt);
                        }
                    }
                }
                UsbCommand::SendRawPacket { raw } => {
                    if let Ok(pkt) = Packet::read_from(raw.as_slice()) {
                        mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                        let _ = mesh.tx_queue.push(pkt);
                    }
                }
                UsbCommand::SendTracePath {
                    tag,
                    auth,
                    flags,
                    path_len,
                    path,
                } => {
                    let hash_bytes = 1u8 << (flags & 0x03);
                    if hash_bytes <= 2 && path_len as usize <= 64 {
                        if let Ok(tp) =
                            crate::meshcore::packets::TracePacket::new(tag, auth, flags, &[])
                        {
                            if let Ok(mut pkt) = tp.to_packet(
                                crate::meshcore::packet::ROUTE_TYPE_DIRECT,
                            ) {
                                let hops = path_len / hash_bytes;
                                pkt.path_len =
                                    ((hash_bytes - 1) << 6) | (hops & 63);
                                pkt.path[..path_len as usize]
                                    .copy_from_slice(&path[..path_len as usize]);
                                mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                                let _ = mesh.tx_queue.push(pkt);
                            }
                        }
                    }
                }
                UsbCommand::SetRepeat(rep) => {
                    info!("Setting repeater mode: {}", rep);
                    mesh.is_repeater = rep;
                }
                UsbCommand::SetFrequency(f) => {
                    info!("Frequency updated to: {} Hz", f);
                    let _ = crate::radio::lora_phy::request_radio_config();
                }
                UsbCommand::SetTxPower(tx) => {
                    info!("TX power updated to: {} dBm", tx);
                    let _ = crate::radio::lora_phy::request_radio_config();
                }
                UsbCommand::SetSpreadingFactor(sf) => {
                    info!("SF updated to: {}", sf);
                    let _ = crate::radio::lora_phy::request_radio_config();
                }
                UsbCommand::SetBandwidth(bw) => {
                    info!("BW updated to: {} kHz", bw);
                    let _ = crate::radio::lora_phy::request_radio_config();
                }
                UsbCommand::SetCodingRate(cr) => {
                    info!("CR updated to: 4/{}", cr);
                    let _ = crate::radio::lora_phy::request_radio_config();
                }
                UsbCommand::SetPathHashMode(mode) => {
                    info!("Setting path.hash.mode to: {} ({} bytes/hop)", mode, mode + 1);
                    mesh.set_path_hash_mode(mode);
                    node.path_hash_mode = mode;
                }
            }
        }

        // Poll semantic input events (buttons, D-pads, gestures)
        if let Some(event) = board.inputs.poll_event().await {
            match event {
                InputEvent::NextPage => {
                    info!("Input: Next Page requested");
                    // Cycle through pages: 0 -> 1 -> 2 -> 0
                    node.current_page = (node.current_page + 1) % 3;
                    page_turns = page_turns.wrapping_add(1);

                    #[cfg(feature = "thinknode-m1")]
                    if let Some(d) = &mut display {
                        if page_turns % EPD_FULL_REFRESH_INTERVAL == 0 {
                            info!("Page turn #{}: performing periodic full screen refresh to clear ghosting", page_turns);
                            d.render_dashboard_full(&node).await.ok();
                        } else {
                            d.render_dashboard(&node).await.ok();
                        }
                    }
                }
                InputEvent::ActionPrimary => {
                    info!("Input: Primary Action (broadcasting MeshCore Advert)...");
                    node.tx_beacons = node.tx_beacons.saturating_add(1);

                    // Quick double-flash on blue LED to give immediate visual feedback
                    for _ in 0..2 {
                        board.telemetry.blue_led.set_high();
                        Timer::after_millis(40).await;
                        board.telemetry.blue_led.set_low();
                        Timer::after_millis(40).await;
                    }

                    // Queue standard MeshCore Advertisement packet over LoRa
                    let _ = mesh.queue_advert(
                        node.epoch_time as u32,
                        node.latitude,
                        node.longitude,
                        true,
                    );
                }
                _ => {}
            }
        }

        // Transmit any pending packets from MeshEngine tx queue (adverts, text, repeats)
        while let Some(out_pkt) = mesh.tx_queue.pop() {
            let mut wire_buf = [0u8; 256];
            if let Ok(wire_len) = out_pkt.write_to(&mut wire_buf) {
                info!("Transmitting MeshCore packet: {} bytes (type: {})", wire_len, out_pkt.payload_type());
                let _ = lora.transmit(&wire_buf[..wire_len]).await;
                lora.start_receive().await;
                crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                });
            }
        }

        Timer::after_millis(20).await;
        tick = tick.wrapping_add(1);

        // After ~60 s of healthy uptime, give a future panic a fresh reset
        // budget (protects against a panic-at-boot reboot loop).
        if tick == 3000 {
            crate::fault::clear_reset_budget();
        }

        // Heartbeat LED flash every 2 seconds
        if tick % 100 == 0 {
            board.telemetry.blue_led.set_high(); // ON
            Timer::after_millis(20).await;
            board.telemetry.blue_led.set_low(); // OFF
        }
    }
}
