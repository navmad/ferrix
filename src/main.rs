#![no_std]
#![no_main]

pub mod boards;
pub mod companion;
pub mod display;
pub mod gps;
pub mod input;
pub mod meshcore;
pub mod radio;
pub mod telemetry;

use defmt::info;
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::peripherals;
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::usb::{self, vbus_detect::HardwareVbusDetect};
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
use crate::radio::{LoraConfig, Sx1262};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    TWISPI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
});

// Static allocations for display framebuffer
#[cfg(feature = "thinknode-m1")]
static EPD_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();

/// Number of page turns between full screen refreshes (clears e-ink ghosting)
const EPD_FULL_REFRESH_INTERVAL: u32 = 9;

/// Custom panic handler: Blinks Red LED (P1.04) continuously to indicate fatal state without hard-faulting
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    defmt::error!("FATAL PANIC OCCURRED!");
    // Directly toggle P1.04 (Red LED) in a loop
    let p1_outset = (0x5000_0800 + 0x508) as *mut u32;
    let p1_outclr = (0x5000_0800 + 0x50C) as *mut u32;
    let p1_dirset = (0x5000_0800 + 0x518) as *mut u32;

    unsafe {
        p1_dirset.write_volatile(1 << 4);
        loop {
            p1_outset.write_volatile(1 << 4);
            cortex_m::asm::delay(2_000_000);
            p1_outclr.write_volatile(1 << 4);
            cortex_m::asm::delay(2_000_000);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = CurrentBoard::init(p);

    // 0. Initialize USB CDC-ACM Serial & 1200-baud DFU Auto-Reset
    let usb_driver = usb::Driver::new(board.usbd, Irqs, HardwareVbusDetect::new(Irqs));
    let (usb_dev, cdc_class) = crate::companion::usb::init_usb(usb_driver);
    spawner.spawn(crate::companion::usb::usb_task(usb_dev)).unwrap();
    spawner.spawn(crate::companion::usb::cdc_acm_task(cdc_class)).unwrap();

    info!("====================================================");
    info!(" MeshCore Companion Firmware (ferrix) ");
    info!(" Active Board: {} ", CurrentBoard::NAME);
    info!(" Runtime: Embassy async no_std (nRF52840 Cortex-M4) ");
    info!("====================================================");

    // Initial node companion state
    let mut node = CompanionNode::new(CurrentBoard::NAME, CurrentBoard::NAME, CurrentBoard::CAPABILITIES);

    // Initialize native MeshCore Routing Engine & Local Cryptographic Identity
    let local_id = LocalIdentity::new(CurrentBoard::NAME, CurrentBoard::NAME);
    let mut mesh = MeshEngine::new(local_id, true);

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

    let mut lora = Sx1262::new(
        spim0,
        board.lora.cs,
        board.lora.reset,
        board.lora.busy,
        board.lora.dio1,
        board.lora.rf_sw,
    );

    let lora_config = LoraConfig::default();

    if lora.init(&lora_config).await.is_ok() {
        info!("LoRa radio online! Starting continuous RX...");
        lora.start_receive().await;
    } else {
        defmt::error!("Failed to initialize LoRa radio!");
    }

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

            // Deserialize and dispatch through MeshCore Network Engine
            if let Ok(pkt) = Packet::read_from(&rx_buf[..len]) {
                match mesh.handle_inbound_packet(pkt) {
                    InboundAction::ReceivedAdvert(adv) => {
                        info!("MeshCore Advert from: {}", adv.callsign.as_str());
                    }
                    InboundAction::ReceivedGroupText(grp) => {
                        info!("MeshCore Group Text [ch {}]: <{}> {}", grp.channel_hash, grp.sender.as_str(), grp.message.as_str());
                        node.unread_messages = node.unread_messages.saturating_add(1);
                    }
                    InboundAction::ReceivedDirectText(msg) => {
                        info!("MeshCore Direct Msg: {}", msg.text.as_str());
                        node.unread_messages = node.unread_messages.saturating_add(1);
                    }
                    InboundAction::ReceivedAck(_ack) => {
                        info!("MeshCore ACK received");
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
                UsbCommand::SetTime(epoch) => {
                    info!("Node epoch time updated to: {}", epoch);
                    node.epoch_time = epoch;
                }
                UsbCommand::TriggerAdvert => {
                    info!("Queuing MeshCore Advert from USB CLI command...");
                    node.tx_beacons = node.tx_beacons.saturating_add(1);
                    let _ = mesh.queue_advert(
                        node.epoch_time as u32,
                        node.latitude,
                        node.longitude,
                        node.battery_percentage,
                    );
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
                        node.battery_percentage,
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
            }
        }

        Timer::after_millis(20).await;
        tick = tick.wrapping_add(1);

        // Heartbeat LED flash every 2 seconds
        if tick % 100 == 0 {
            board.telemetry.blue_led.set_high(); // ON
            Timer::after_millis(20).await;
            board.telemetry.blue_led.set_low(); // OFF
        }
    }
}
