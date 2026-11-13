#![no_std]
#![no_main]

pub mod boards;
pub mod companion;
pub mod display;
pub mod gps;
pub mod input;
pub mod meshcore;
pub mod radio;
pub mod storage;
pub mod telemetry;

use defmt::info;
use defmt_rtt as _;
use embassy_futures::select::{select4, Either4};

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::peripherals;
use embassy_nrf::saadc;
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::usb;
use embassy_time::{Duration, Ticker, Timer};
#[cfg(feature = "thinknode-m1")]
use static_cell::StaticCell;

use crate::boards::{Board, CurrentBoard};
use crate::companion::protocol::CompanionNode;
use crate::companion::usb::{UsbCommand, USB_COMMAND_CHANNEL};
#[cfg(feature = "thinknode-m1")]
use crate::display::{EpdFrameBuffer, NodeDisplay, Ssd1681, ThinkNodeDisplay};
use crate::input::{InputEvent, INPUT_EVENT_CHANNEL};
use crate::meshcore::{InboundAction, LocalIdentity, MeshEngine, Packet};
use crate::radio::{lora_phy_task, LoraConfig, LoraPhyChannelRadio, LoraRadio};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    TWISPI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => saadc::InterruptHandler;
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

/// Tell the app that a contact was discovered (full record) or re-advertised.
fn push_new_advert(ct: &crate::meshcore::mesh::DiscoveredContact) {
    let record = crate::companion::protocol::serialize_contact(ct);
    let mut frame = heapless::Vec::<u8, 256>::new();
    let _ = frame.push(crate::companion::protocol::PUSH_CODE_NEW_ADVERT);
    let _ = frame.extend_from_slice(&record[1..]);
    let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(frame);
}

fn push_advert(pub_key: &[u8; 32]) {
    let mut frame = heapless::Vec::<u8, 256>::new();
    let _ = frame.push(crate::companion::protocol::PUSH_CODE_ADVERT);
    let _ = frame.extend_from_slice(pub_key);
    let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(frame);
}

fn push_path_updated(pub_key: &[u8; 32]) {
    let mut frame = heapless::Vec::<u8, 256>::new();
    let _ = frame.push(crate::companion::protocol::PUSH_CODE_PATH_UPDATED);
    let _ = frame.extend_from_slice(pub_key);
    let _ = crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(frame);
}

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

/// Seconds the display frontlight stays on after the last button press.
const BACKLIGHT_IDLE_SECS: u64 = 15;

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

/// Custom panic handler: 3 quick blue blinks, repeated forever.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    fatal_blink_forever(3)
}

/// HardFault handler: 6 quick blue blinks, repeated forever.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    fatal_blink_forever(6)
}

/// Drive the blue status LED unless the user disabled LEDs in the UI.
fn set_blue_led(led: &mut embassy_nrf::gpio::Output<'static>, on: bool) {
    if on && !crate::companion::usb::leds_disabled() {
        led.set_high();
    } else {
        led.set_low();
    }
}

async fn advert_feedback(led: &mut embassy_nrf::gpio::Output<'static>) {
    for _ in 0..2 {
        set_blue_led(led, true);
        Timer::after_millis(40).await;
        set_blue_led(led, false);
        Timer::after_millis(40).await;
    }
}

fn tx_duty_pct(af_tenths: u8) -> i64 {
    1000 / (af_tenths as i64 + 10)
}

fn tx_budget_max(af_tenths: u8) -> i64 {
    // One-hour duty-cycle window.
    3_600_000 * tx_duty_pct(af_tenths) / 100
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
    embassy_nrf::interrupt::SAADC.set_priority(Priority::P3);

    let mut board = CurrentBoard::init(p);

    // Keep the GPS module held in reset until the user enables it from the
    // dashboard. P1.02 is GPS_EN, P1.05 is the active-low reset.
    let mut gps = board.gps;
    if let Some(g) = gps.as_mut() {
        g.standby.set_low();
        g.reinit.set_low();
    }

    // Battery ADC: ThinkNode M1 AIN2 (P0.04), 150K+150K divider (2:1).
    // `single_ended` defaults to gain 1/6 and the internal 0.6 V reference,
    // which is the 3.6 V full-scale range the board's multiplier assumes.
    let mut battery_adc = saadc::Saadc::new(
        board.saadc,
        Irqs,
        saadc::Config::default(),
        [saadc::ChannelConfig::single_ended(board.bat_adc_pin)],
    );

    // Take one sample before the companion/display start so a freshly booted
    // device never reports 0 mV (and therefore 0%) to the app.
    let mut battery_buf = [0i16; 1];
    if CurrentBoard::HAS_BATTERY_ADC {
        battery_adc.sample(&mut battery_buf).await;
        let raw = battery_buf[0].max(0) as u16;
        crate::telemetry::set_battery_millivolts(
            crate::telemetry::adc_raw_to_battery_millivolts(raw),
        );
    }

    // Initialize persistent configuration from flash
    let (node_name, custom_key) = crate::storage::init_config();

    // Load the contact and message snapshots into RAM.
    crate::storage::load();

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
    // Publish the identity so every transport (USB, BLE) reads the same keys.
    crate::companion::usb::set_identity(pub_key, priv_key);
    let mut mesh = MeshEngine::new(local_id, false);
    // Forwarding is enabled for a dedicated repeater role or off-grid mode.
    mesh.is_repeater = crate::companion::usb::forwarding_enabled();

    // 0. Initialize USB CDC-ACM Serial & 1200-baud DFU Auto-Reset FIRST
    // This guarantees serial CLI, packet logging, and DFU auto-reset are always available.
    let vbus: &'static embassy_nrf::usb::vbus_detect::SoftwareVbusDetect =
        &*VBUS_DETECT.init(embassy_nrf::usb::vbus_detect::SoftwareVbusDetect::new(true, true));
    let usb_driver = usb::Driver::new(board.usbd, Irqs, vbus);
    let (usb_dev, cdc_class) = crate::companion::usb::init_usb(usb_driver);
    spawner.spawn(crate::companion::usb::usb_task(usb_dev)).unwrap();
    spawner.spawn(crate::companion::usb::cdc_acm_task(cdc_class, pub_key, priv_key)).unwrap();
    spawner.spawn(crate::companion::ble::contact_task()).unwrap();
    spawner
        .spawn(crate::boards::thinknode_m1::thinknode_input_task(board.inputs))
        .unwrap();

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
    info!(" Ferrix Companion Firmware (ferrix) ");
    info!(" Active Board: {} ", CurrentBoard::NAME);
    info!(" Runtime: Embassy async no_std (nRF52840 Cortex-M4) ");
    info!("====================================================");

    // Visual Boot Confirmation: 3 fast blue blinks indicate successful MCU & runtime start
    for _ in 0..3 {
        set_blue_led(&mut board.telemetry.blue_led, true); // ON (active-high)
        Timer::after_millis(70).await;
        set_blue_led(&mut board.telemetry.blue_led, false); // OFF
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
    set_blue_led(&mut board.telemetry.blue_led, false);

    let mut rx_buf = [0u8; 256];
    let mut page_turns = 0u32;

    // The frontlight starts on at boot; turn it off once the user stops
    // pressing buttons.
    #[cfg(feature = "thinknode-m1")]
    let mut backlight_on = true;
    #[cfg(feature = "thinknode-m1")]
    let mut epd_active = true;
    #[cfg(feature = "thinknode-m1")]
    let mut backlight_deadline = embassy_time::Instant::now()
        .checked_add(embassy_time::Duration::from_secs(BACKLIGHT_IDLE_SECS))
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_secs(2));
    let mut tick: u32 = 0;

    let af0 = crate::companion::usb::RADIO_SETTINGS
        .lock(|r| r.borrow().airtime_factor_tenths);
    let mut tx_budget_ms = tx_budget_max(af0);
    let mut tx_budget_last = embassy_time::Instant::now();

    loop {
        let wake = select4(
            lora.receive(&mut rx_buf),
            USB_COMMAND_CHANNEL.receive(),
            INPUT_EVENT_CHANNEL.receive(),
            ticker.next(),
        )
        .await;

        match wake {
        Either4::First((len, rssi, snr)) => {
            info!("Received LoRa packet: {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
            set_blue_led(&mut board.telemetry.blue_led, true); // ON
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
            let rx_air = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                let s = r.borrow();
                crate::radio::estimate_airtime_ms(
                    len,
                    s.sf,
                    s.bw_khz,
                    s.cr,
                    if s.sf <= 8 { 32 } else { 16 },
                )
            });
            crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.rx_packets = tel.rx_packets.saturating_add(1);
                tel.rx_air_ms = tel.rx_air_ms.saturating_add(rx_air);
                tel.last_rssi = rssi;
                tel.last_snr = snr;
                tel.has_rx = true;
            });

            // Deserialize and dispatch through MeshCore Network Engine
            if let Ok(pkt) = Packet::read_from(&rx_buf[..len]) {
                let advert_hops = pkt.path_hash_count() as u8;
                // Retain the signed payload so the contact can be exported.
                let advert_payload =
                    if pkt.payload_type() == crate::meshcore::packet::PAYLOAD_TYPE_ADVERT {
                        let mut v = heapless::Vec::<u8, 184>::new();
                        let _ = v.extend_from_slice(&pkt.payload[..pkt.payload_len]);
                        Some(v)
                    } else {
                        None
                    };
                match mesh.handle_inbound_packet(pkt) {
                    InboundAction::ReceivedAdvert(adv) => {
                        info!("MeshCore Advert from: {}", adv.name.as_str());
                        let node_id = [adv.pub_key[0], adv.pub_key[1], adv.pub_key[2], adv.pub_key[3]];

                        // Check if this contact already exists in our list
                        let contact_type = CompanionNode::get_contact_type_from_flags(adv.flags);
                        let should_add = node.should_auto_add_contact_type(contact_type)
                            && node.autoadd_allows_hops(advert_hops);

                        let mut store_blob = false;
                        crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                            let mut list = c.borrow_mut();
                            if let Some(existing) = list.iter_mut().find(|ct| ct.pub_key == adv.pub_key) {
                                // Anti-replay: ignore an advert no newer than the last accepted one.
                                if existing.last_advert_ts != 0 && adv.timestamp <= existing.last_advert_ts {
                                    return;
                                }
                                // Only rewrite the shared advert blob when the
                                // contact's advertised identity actually changed.
                                if existing.callsign != adv.name
                                    || existing.contact_type != contact_type
                                    || existing.latitude != adv.latitude
                                    || existing.longitude != adv.longitude
                                    || !crate::storage::adverts::has_blob(&adv.pub_key)
                                {
                                    store_blob = true;
                                }
                                existing.callsign = adv.name.clone();
                                existing.last_seen_epoch = node.epoch_time as u32;
                                existing.last_advert_ts = adv.timestamp;
                                existing.rssi = rssi;
                                existing.snr = snr;
                                existing.latitude = adv.latitude;
                                existing.longitude = adv.longitude;
                                existing.contact_type = contact_type;
                                // Contact flags (favorite/telemetry permissions) are
                                // independent of the advert flags; keep the stored ones.
                                push_advert(&adv.pub_key);
                                return;
                            }

                            // New contact. The app is told even when auto-add
                            // declines it, matching the reference discovery flow.
                            let mut cs = heapless::String::new();
                            let _ = cs.push_str(adv.name.as_str());
                            let discovered = crate::meshcore::mesh::DiscoveredContact {
                                pub_key: adv.pub_key,
                                callsign: cs,
                                node_id,
                                last_seen_epoch: node.epoch_time as u32,
                                rssi,
                                snr,
                                latitude: adv.latitude,
                                longitude: adv.longitude,
                                // Contact flags start cleared; the advert flags
                                // only classified `contact_type` above.
                                flags: 0,
                                contact_type,
                                out_path: [0u8; 64],
                                out_path_len: 0xFF,
                                last_advert_ts: adv.timestamp,
                            };

                            let mut added = false;
                            if should_add {
                                if list.len() < list.capacity() {
                                    let _ = list.push(discovered.clone());
                                    added = true;
                                } else if node.should_overwrite_when_full() {
                                    // Find the oldest non-favorite contact to overwrite.
                                    let mut oldest_idx = None;
                                    let mut oldest_lastmod = u32::MAX;
                                    for (idx, ct) in list.iter().enumerate() {
                                        let is_favorite = (ct.flags
                                            & crate::companion::protocol::CONTACT_FLAG_FAVORITE)
                                            != 0;
                                        if !is_favorite && ct.last_seen_epoch < oldest_lastmod {
                                            oldest_lastmod = ct.last_seen_epoch;
                                            oldest_idx = Some(idx);
                                        }
                                    }
                                    if let Some(idx) = oldest_idx {
                                        let overwritten = list[idx].pub_key;
                                        let mut push = heapless::Vec::<u8, 256>::new();
                                        let _ = push.push(
                                            crate::companion::protocol::PUSH_CODE_CONTACT_DELETED,
                                        );
                                        let _ = push.extend_from_slice(&overwritten);
                                        let _ =
                                            crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(push);
                                        crate::storage::adverts::forget(&overwritten);
                                        list[idx] = discovered.clone();
                                        added = true;
                                    }
                                }
                            }

                            if added {
                                store_blob = true;
                            }
                            push_new_advert(&discovered);

                            if !added && list.len() >= list.capacity() {
                                let mut push = heapless::Vec::<u8, 256>::new();
                                let _ = push.push(
                                    crate::companion::protocol::PUSH_CODE_CONTACTS_FULL,
                                );
                                let _ =
                                    crate::companion::ble::COMPANION_PUSH_CHANNEL.try_send(push);
                            }
                        });
                        if store_blob {
                            if let Some(payload) = advert_payload.as_ref() {
                                crate::storage::adverts::remember(&adv.pub_key, payload);
                            }
                        }
                        crate::storage::request_contacts();
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
                        crate::storage::request_messages();

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
                        crate::storage::request_messages();

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
                        let path_updated = crate::companion::usb::DISCOVERED_CONTACTS
                            .lock(|c| {
                                if let Some(ct) =
                                    c.borrow_mut().iter_mut().find(|ct| ct.pub_key == sender_pub)
                                {
                                    ct.out_path = path.path;
                                    ct.out_path_len = path.path_len;
                                    true
                                } else {
                                    false
                                }
                            });
                        if path_updated {
                            push_path_updated(&sender_pub);
                        }
                        crate::storage::request_contacts();
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
            set_blue_led(&mut board.telemetry.blue_led, false); // OFF
        }
        Either4::Second(cmd) => {
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
                        crate::companion::usb::set_identity(*mesh.identity.pub_key(), priv_key);
                        // Idempotent for the companion-app path (which already
                        // persisted it) and required for any other caller.
                        crate::storage::set_custom_priv_key(priv_key);
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
                    // Moving off an off-grid band can disable client repeat.
                    mesh.is_repeater = crate::companion::usb::forwarding_enabled();
                    let _ = crate::radio::lora_phy::request_radio_config();
                }
                UsbCommand::SetRadio {
                    frequency_hz,
                    bw_khz,
                    sf,
                    cr,
                } => {
                    info!(
                        "Radio updated: {} Hz, BW {} kHz, SF{}, CR 4/{}",
                        frequency_hz, bw_khz, sf, cr
                    );
                    crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                        let mut s = r.borrow_mut();
                        s.frequency_hz = frequency_hz;
                        s.bw_khz = bw_khz;
                        s.sf = sf;
                        s.cr = cr;
                        s.offgrid_home_freq = 0;
                        if !crate::companion::usb::is_offgrid_freq_hz(frequency_hz) {
                            s.client_repeat = false;
                        }
                    });
                    mesh.is_repeater = crate::companion::usb::forwarding_enabled();
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
        Either4::Third(event) => {
            // Any interaction wakes the frontlight and restarts its idle timer.
            #[cfg(feature = "thinknode-m1")]
            {
                backlight_on = true;
                epd_active = true;
                backlight_deadline = embassy_time::Instant::now()
                    .checked_add(embassy_time::Duration::from_secs(BACKLIGHT_IDLE_SECS))
                    .unwrap();
                if let Some(d) = &mut display {
                    d.set_backlight(true);
                }
            }

            match event {
                InputEvent::NextPage => {
                    info!("Input: Next Page requested");
                    // Cycle through pages: 0 -> 1 -> 2 -> 0
                    node.current_page = (node.current_page + 1) % crate::display::ui::PAGE_COUNT;
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
                    match node.current_page {
                        0 => {
                            info!("Input: flood advert");
                            node.tx_beacons = node.tx_beacons.saturating_add(1);
                            advert_feedback(&mut board.telemetry.blue_led).await;
                            let _ = mesh.queue_advert(
                                node.epoch_time as u32,
                                node.latitude,
                                node.longitude,
                                true,
                            );
                        }
                        1 => {
                            info!("Input: zero-hop advert");
                            node.tx_beacons = node.tx_beacons.saturating_add(1);
                            advert_feedback(&mut board.telemetry.blue_led).await;
                            let _ = mesh.queue_advert(
                                node.epoch_time as u32,
                                node.latitude,
                                node.longitude,
                                false,
                            );
                        }
                        2 => {
                            let next = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                s.tx_power_dbm = match s.tx_power_dbm {
                                    22 => 17,
                                    17 => 14,
                                    14 => 10,
                                    _ => 22,
                                };
                                s.tx_power_dbm
                            });
                            crate::storage::request_save_config();
                            let _ = crate::radio::lora_phy::request_radio_config();
                            info!("Input: TX power -> {} dBm", next);
                        }
                        3 => {
                            crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                                let mut t = t.borrow_mut();
                                t.tx_beacons = 0;
                                t.rx_packets = 0;
                            });
                            node.tx_beacons = 0;
                            node.rx_packets = 0;
                            info!("Input: counters reset");
                        }
                        4 => {
                            let on = !crate::gps::gps_enabled();
                            if let Some(g) = gps.as_mut() {
                                if on {
                                    g.standby.set_high();
                                    g.reinit.set_high();
                                } else {
                                    g.standby.set_low();
                                    g.reinit.set_low();
                                }
                            }
                            crate::gps::set_gps_enabled(on);
                            info!("Input: GPS power {}", if on { "on" } else { "off" });
                        }
                        5 => {
                            let active = crate::companion::usb::offgrid_active();
                            let change = crate::companion::usb::offgrid_set(!active);
                            if change.no_band {
                                info!("Input: no off-grid frequency in this band");
                            } else {
                                if change.changed {
                                    crate::storage::request_save_config();
                                }
                                if change.freq_changed {
                                    let _ = crate::radio::lora_phy::request_radio_config();
                                }
                                mesh.is_repeater = crate::companion::usb::forwarding_enabled();
                                info!("Input: offgrid {}", if active { "off" } else { "on" });
                            }
                        }
                        6 => {
                            let disabled = !crate::companion::usb::leds_disabled();
                            crate::companion::usb::set_leds_disabled(disabled);
                            set_blue_led(&mut board.telemetry.blue_led, !disabled);
                            info!(
                                "Input: blue LED {}",
                                if disabled { "disabled" } else { "enabled" }
                            );
                        }
                        _ => {
                            let on = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                s.rx_duty_cycle = !s.rx_duty_cycle;
                                s.rx_duty_cycle
                            });
                            crate::storage::request_save_config();
                            let _ = crate::radio::lora_phy::request_radio_config();
                            info!("Input: RX duty cycle {}", if on { "on" } else { "off" });
                        }
                    }

                    #[cfg(feature = "thinknode-m1")]
                    if let Some(d) = &mut display {
                        d.render_dashboard(&node).await.ok();
                    }
                }
                _ => {}
            }
        }
        Either4::Fourth(_) => {
            tick = tick.wrapping_add(1);

            // Turn the frontlight off after 15 s of no button presses.
            #[cfg(feature = "thinknode-m1")]
            if backlight_on && embassy_time::Instant::now() >= backlight_deadline {
                if let Some(d) = &mut display {
                    d.set_backlight(false);
                }
                backlight_on = false;
                epd_active = false;
            }

            // Refresh the dashboard every 60 s (30 ticks) only while the UI
            // is awake. After the backlight timeout the panel is left alone
            // until a button press wakes it again.
            if tick % 30 == 0 {
                #[cfg(feature = "thinknode-m1")]
                if epd_active {
                    if let Some(d) = &mut display {
                        d.render_dashboard(&node).await.ok();
                    }
                }
            }

            // Sample the battery every 10 s (5 ticks).
            if tick % 5 == 0 && CurrentBoard::HAS_BATTERY_ADC {
                battery_adc.sample(&mut battery_buf).await;
                let raw = battery_buf[0].max(0) as u16;
                let mv = crate::telemetry::adc_raw_to_battery_millivolts(raw);
                if mv >= crate::telemetry::MIN_PLAUSIBLE_BATTERY_MV {
                    crate::telemetry::set_battery_millivolts(mv);
                    node.battery_millivolts = mv;
                    node.battery_percentage = crate::telemetry::millivolts_to_percentage(mv);
                }
            }

            // Heartbeat LED flash every 2 s (each tick), unless disabled.
            if !crate::companion::usb::leds_disabled() {
                set_blue_led(&mut board.telemetry.blue_led, true); // ON
                Timer::after_millis(20).await;
            }
            set_blue_led(&mut board.telemetry.blue_led, false); // OFF
        }
        }

        // Transmit pending packets, subject to the airtime duty-cycle budget.
        let (af_tenths, sf, bw_khz, cr) = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (s.airtime_factor_tenths, s.sf, s.bw_khz, s.cr)
        });
        let duty_pct = tx_duty_pct(af_tenths);
        let max_budget = tx_budget_max(af_tenths);
        let now = embassy_time::Instant::now();
        let elapsed_ms = (now - tx_budget_last).as_millis() as i64;
        if elapsed_ms > 0 {
            tx_budget_ms = (tx_budget_ms + elapsed_ms * duty_pct / 100).min(max_budget);
            tx_budget_last = now;
        }
        tx_budget_ms = tx_budget_ms.min(max_budget);
        let preamble = if sf <= 8 { 32 } else { 16 };

        loop {
            let airtime = match mesh.tx_queue.front() {
                Some(pkt) => {
                    let mut probe = [0u8; 256];
                    match pkt.write_to(&mut probe) {
                        Ok(len) => crate::radio::estimate_airtime_ms(
                            len, sf, bw_khz, cr, preamble,
                        ),
                        Err(_) => 0,
                    }
                }
                None => break,
            };

            if duty_pct < 100 && airtime > 0 && tx_budget_ms < airtime as i64 {
                break;
            }

            let Some(out_pkt) = mesh.tx_queue.pop() else {
                break;
            };
            let mut wire_buf = [0u8; 256];
            if let Ok(wire_len) = out_pkt.write_to(&mut wire_buf) {
                info!(
                    "Transmitting MeshCore packet: {} bytes (type: {})",
                    wire_len,
                    out_pkt.payload_type()
                );
                let _ = lora.transmit(&wire_buf[..wire_len]).await;
                lora.start_receive().await;
                crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                    tel.tx_air_ms = tel.tx_air_ms.saturating_add(airtime);
                });
            }
            if duty_pct < 100 && airtime > 0 {
                tx_budget_ms = (tx_budget_ms - airtime as i64).max(0);
            }
        }
    }
}
