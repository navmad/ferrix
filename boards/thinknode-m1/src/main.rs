#![no_std]
#![no_main]

pub mod app;
pub mod board;
pub mod companion;
pub mod display;
pub mod gps;
pub mod host;
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
use embassy_nrf::saadc;
use embassy_nrf::spim::{self, Spim};
use embassy_nrf::usb;
use embassy_time::{Duration, Timer};
#[cfg(feature = "thinknode-m1")]
use static_cell::StaticCell;

use crate::app::{set_blue_led, ThinkNodeApp};
use ferrix_core::companion::protocol::CompanionNode;
use crate::host::BOARD_HOST;
#[cfg(feature = "thinknode-m1")]
use crate::display::{EpdFrameBuffer, NodeDisplay, Ssd1681, ThinkNodeDisplay};
use crate::meshcore::{LocalIdentity, MeshEngine};
use crate::radio::{sx1262_task, LoraConfig, ChannelRadio, LoraRadio};

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    TWISPI0 => spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => saadc::InterruptHandler;
});

static VBUS_DETECT: StaticCell<embassy_nrf::usb::vbus_detect::SoftwareVbusDetect> = StaticCell::new();

// Static allocations for display framebuffer
#[cfg(feature = "thinknode-m1")]
static EPD_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();

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

/// Forward deferred protocol pushes to the connected companion transport.
#[embassy_executor::task]
async fn delayed_push_task() -> ! {
    loop {
        let frame = ferrix_core::companion::protocol::next_delayed_push().await;
        let _ = ferrix_core::command::COMPANION_PUSH_CHANNEL.try_send(frame);
    }
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
    embassy_nrf::interrupt::QSPI.set_priority(Priority::P3);
    embassy_nrf::interrupt::UARTE0.set_priority(Priority::P3);

    let mut board = crate::board::init(p);
    let qspi = board.qspi;

    // The GPS task owns the L76K UART and control pins; it holds the module
    // in reset/sleep until the dashboard enables it.
    if let Some(gps) = board.gps.take() {
        spawner.spawn(crate::gps::gps_task(board.uarte0, gps)).unwrap();
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
    if crate::board::HAS_BATTERY_ADC {
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
            LocalIdentity::from_seed(node_name.as_str(), &crate::board::hardware_seed())
        })
    } else {
        LocalIdentity::from_seed(node_name.as_str(), &crate::board::hardware_seed())
    };
    let pub_key = local_id.pub_key_bytes();
    let priv_key = local_id.private_key_bytes();
    // Publish the identity so every transport (USB, BLE) reads the same keys.
    crate::companion::usb::set_identity(pub_key, priv_key);
    let mut mesh = MeshEngine::new(
        local_id,
        false,
        &crate::meshcore::BOARD_ENGINE_CONTEXT,
    );
    // Forwarding is enabled for a dedicated repeater role or off-grid mode.
    mesh.is_repeater = crate::companion::usb::forwarding_enabled();

    // 0. Initialize USB CDC-ACM Serial & 1200-baud DFU Auto-Reset FIRST
    // This guarantees serial CLI, packet logging, and DFU auto-reset are always available.
    let vbus: &'static embassy_nrf::usb::vbus_detect::SoftwareVbusDetect =
        &*VBUS_DETECT.init(embassy_nrf::usb::vbus_detect::SoftwareVbusDetect::new(true, true));
    let usb_driver = usb::Driver::new(board.usbd, Irqs, vbus);
    let (usb_dev, cdc_class) = crate::companion::usb::init_usb(
        usb_driver,
        0x239A,
        0x8029,
        "MeshCore",
        "ThinkNode M1 Companion",
    );
    spawner.spawn(crate::companion::usb::usb_task(usb_dev)).unwrap();
    spawner
        .spawn(crate::companion::usb::cdc_acm_task(
            cdc_class,
            pub_key,
            priv_key,
            crate::board::NAME,
            crate::board::CAPABILITIES,
            &BOARD_HOST,
        ))
        .unwrap();
    spawner.spawn(crate::companion::ble::contact_task()).unwrap();
    spawner.spawn(delayed_push_task()).unwrap();
    spawner
        .spawn(crate::board::thinknode_input_task(board.inputs))
        .unwrap();

    // Initial node companion state
    let mut node = CompanionNode::new(
        node_name.as_str(),
        crate::board::NAME,
        crate::board::CAPABILITIES,
        pub_key,
        priv_key,
        &BOARD_HOST,
    );

    // Initialize BLE Companion peripheral (Nordic UART Service)
    crate::companion::ble::init_ble_companion(
        spawner,
        pub_key,
        priv_key,
        crate::board::NAME,
        crate::board::CAPABILITIES,
        "ThinkNode M1",
        &BOARD_HOST,
        qspi,
        crate::fatal_blink_forever,
        crate::storage::spawn_storage,
    );

    info!("====================================================");
    info!(" Ferrix Companion Firmware (ferrix) ");
    info!(" Active Board: {} ", crate::board::NAME);
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
    spawner.spawn(sx1262_task(
        spim0,
        board.lora.cs,
        board.lora.reset,
        board.lora.busy,
        board.lora.dio1,
        board.lora.rf_sw,
    )).unwrap();

    let mut lora = ChannelRadio::new();
    let _ = lora.init(&LoraConfig::default()).await;
    lora.start_receive().await;

    // 2. Initialize Display via generic NodeDisplay trait
    #[cfg(feature = "thinknode-m1")]
    let display = if crate::board::HAS_DISPLAY {
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

            info!("Initializing display and drawing Ferrix boot splash...");
            d.init().await.ok();
            d.render_boot(&node).await.ok();
            Timer::after(Duration::from_secs(2)).await;
            // Presses during the splash are intentionally ignored.
            while ferrix_runtime::app::INPUT_EVENT_CHANNEL.try_receive().is_ok() {}
            // The splash was a full refresh; the first page only needs a
            // differential update.
            d.render_dashboard(
                &node,
                &ferrix_runtime::app::UiState::default(),
            )
            .await
            .ok();
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

    let mut app = ThinkNodeApp::new(
        board.telemetry.blue_led,
        display,
        battery_adc,
    );
    ferrix_runtime::app::run(&mut app, &mut mesh, &mut node, &mut lora).await;

}
