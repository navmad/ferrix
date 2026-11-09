#![no_std]
#![no_main]

pub mod pins;
pub mod radio;
pub mod companion;
pub mod display;
pub mod gps;
pub mod telemetry;

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_nrf::bind_interrupts;
use embassy_nrf::peripherals;
use embassy_nrf::spim::{self, Spim};
use embassy_time::Timer;
use static_cell::StaticCell;

use crate::companion::protocol::CompanionNode;
use crate::display::{render_dashboard, EpdFrameBuffer, Ssd1681};
use crate::radio::{Sx1262, Sx1262Config};

bind_interrupts!(struct Irqs {
    TWISPI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
});

// Static allocations for peripherals
static EPD_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = pins::BoardPins::init(p);

    info!("====================================================");
    info!(" Elecrow ThinkNode M1 - MeshCore Companion Firmware ");
    info!(" Runtime: Embassy async no_std (nRF52840 Cortex-M4) ");
    info!("====================================================");

    // Initial node companion state
    let mut node = CompanionNode::new("THINKNODE-M1");

    // Initialize Blue status LED (active low)
    board.telemetry.blue_led.set_low();

    // 1. Initialize SPI0 for SX1262 LoRa Radio (8 MHz)
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

    let lora_config = Sx1262Config {
        frequency_hz: 915_000_000, // 915 MHz US mesh band
        tx_power_dbm: 22,
        spreading_factor: 9,
        bandwidth_khz: 125,
        coding_rate: 1,
        preamble_length: 16,
    };

    if lora.init(&lora_config).await.is_ok() {
        info!("LoRa radio online! Starting continuous RX...");
        lora.start_receive().await;
    } else {
        defmt::error!("Failed to initialize SX1262 radio!");
    }

    // 2. Initialize SPI1 for SSD1681 1.54" E-Paper Display (4 MHz)
    let mut spim1_config = spim::Config::default();
    spim1_config.frequency = spim::Frequency::M4;
    spim1_config.mode = spim::MODE_0;

    let spim1 = Spim::new_txonly(
        board.spim1,
        Irqs,
        board.display.sclk,
        board.display.mosi,
        spim1_config,
    );

    let mut epd = Ssd1681::new(
        spim1,
        board.display.cs,
        board.display.dc,
        board.display.reset,
        board.display.busy,
        board.display.backlight,
    );

    let fb = EPD_FRAMEBUFFER.init(EpdFrameBuffer::new());

    info!("Initializing E-Paper Display and drawing MeshCore Dashboard...");
    epd.init().await;
    render_dashboard(&node, fb);
    epd.display_frame(fb).await;
    info!("Dashboard rendered on 1.54\" E-Paper display!");

    // Turn blue LED off after boot
    board.telemetry.blue_led.set_high();

    let mut rx_buf = [0u8; 256];
    let mut tick = 0u32;

    loop {
        // Poll for incoming LoRa packets
        if let Some((len, rssi, snr)) = lora.poll_receive(&mut rx_buf).await {
            info!("Received LoRa packet: {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
            board.telemetry.blue_led.set_low();
            Timer::after_millis(50).await;
            board.telemetry.blue_led.set_high();
        }

        // Check buttons
        if board.buttons.function_btn.is_low() {
            info!("Function button (FK) pressed! Sending self advertisement ping...");
            board.telemetry.blue_led.set_low();
            let ping_packet = b"MESHCORE_BEACON:THINKNODE-M1";
            let _ = lora.transmit(ping_packet).await;
            lora.start_receive().await;
            board.telemetry.blue_led.set_high();
            Timer::after_millis(300).await; // Debounce
        }

        if board.buttons.page_btn.is_low() {
            info!("Page turn button (TK) pressed! Refreshing display...");
            node.unread_messages = node.unread_messages.saturating_add(1);
            render_dashboard(&node, fb);
            epd.display_frame(fb).await;
            Timer::after_millis(300).await; // Debounce
        }

        Timer::after_millis(50).await;
        tick = tick.wrapping_add(1);

        // Heartbeat LED flash every 2 seconds
        if tick % 40 == 0 {
            board.telemetry.blue_led.set_low();
            Timer::after_millis(15).await;
            board.telemetry.blue_led.set_high();
        }
    }
}
