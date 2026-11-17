//! Common nRF52 boot sequence for SoftDevice-based boards.
//!
//! Every board using the S140 SoftDevice has to reserve interrupt priorities
//! and capture the reset reason before anything else runs. Keeping that here
//! means a new board only supplies pins and device configuration.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_nrf::interrupt::{InterruptExt, Priority};

static DCDC_REG1_ENABLED: AtomicBool = AtomicBool::new(false);
static LFCLK_MODE: AtomicU8 = AtomicU8::new(0);

/// Sleep-clock accuracy for an external 32.768 kHz crystal (`LFXO`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LfclkAccuracy {
    Ppm20,
    #[default]
    Ppm50,
    Ppm100,
    Ppm250,
}

/// Low-frequency clock (`32.768 kHz`) source for both `RTC1` (`embassy-time`)
/// and the S140 SoftDevice.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Lfclk {
    /// Internal `32.768 kHz` RC oscillator (`500 ppm`, periodic calibration).
    /// Safe fallback for boards without an external `32.768 kHz` crystal.
    #[default]
    InternalRc,
    /// External `32.768 kHz` crystal oscillator (`LFXO`).
    /// Only enable when the board populates the crystal on `XL1`/`XL2`.
    ExternalXtal(LfclkAccuracy),
}

impl Lfclk {
    const fn to_u8(self) -> u8 {
        match self {
            Self::InternalRc => 0,
            Self::ExternalXtal(LfclkAccuracy::Ppm20) => 1,
            Self::ExternalXtal(LfclkAccuracy::Ppm50) => 2,
            Self::ExternalXtal(LfclkAccuracy::Ppm100) => 3,
            Self::ExternalXtal(LfclkAccuracy::Ppm250) => 4,
        }
    }

    const fn from_u8(raw: u8) -> Self {
        match raw {
            1 => Self::ExternalXtal(LfclkAccuracy::Ppm20),
            2 => Self::ExternalXtal(LfclkAccuracy::Ppm50),
            3 => Self::ExternalXtal(LfclkAccuracy::Ppm100),
            4 => Self::ExternalXtal(LfclkAccuracy::Ppm250),
            _ => Self::InternalRc,
        }
    }
}

/// Board-level nRF52 startup hardware options.
///
/// Defaults are conservative so boards without external inductors or crystals
/// boot safely; boards with those components opt in via [`init_with`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StartupConfig {
    /// Enable the internal stage-1 buck DC-DC regulator (`REG1`, VDD -> DEC4).
    /// Only enable when the board populates the external `DCC`/`DEC4` inductor.
    pub dcdc_reg1: bool,
    /// `32.768 kHz` low-frequency clock source.
    pub lfclk: Lfclk,
}

/// Returns whether [`init_with`] enabled the `REG1` buck DC-DC regulator.
pub fn dcdc_reg1_enabled() -> bool {
    DCDC_REG1_ENABLED.load(Ordering::Relaxed)
}

/// Returns the `32.768 kHz` LFCLK configuration chosen at startup.
pub fn lfclk_config() -> Lfclk {
    Lfclk::from_u8(LFCLK_MODE.load(Ordering::Relaxed))
}

/// Initialise the nRF peripherals with SoftDevice-safe interrupt priorities
/// and default (`LDO` + `InternalRc`) hardware configuration.
pub fn init() -> embassy_nrf::Peripherals {
    init_with(StartupConfig::default())
}

/// Initialise the nRF peripherals with SoftDevice-safe interrupt priorities
/// and board-specific startup options.
///
/// The S140 stack reserves priorities 0, 1 and 4, so Embassy system and
/// peripheral interrupts are placed on 2 and 3.
pub fn init_with(opts: StartupConfig) -> embassy_nrf::Peripherals {
    DCDC_REG1_ENABLED.store(opts.dcdc_reg1, Ordering::Relaxed);
    LFCLK_MODE.store(opts.lfclk.to_u8(), Ordering::Relaxed);

    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    config.dcdc.reg1 = opts.dcdc_reg1;
    config.lfclk_source = match opts.lfclk {
        Lfclk::InternalRc => embassy_nrf::config::LfclkSource::InternalRC,
        Lfclk::ExternalXtal(_) => embassy_nrf::config::LfclkSource::ExternalXtal,
    };
    let peripherals = embassy_nrf::init(config);

    embassy_nrf::interrupt::USBD.set_priority(Priority::P3);
    embassy_nrf::interrupt::TWISPI0.set_priority(Priority::P3);
    embassy_nrf::interrupt::TWISPI1.set_priority(Priority::P3);
    embassy_nrf::interrupt::SAADC.set_priority(Priority::P3);
    embassy_nrf::interrupt::QSPI.set_priority(Priority::P3);
    embassy_nrf::interrupt::UARTE0.set_priority(Priority::P3);

    // POWER.RESETREAS is sticky; capture it before a later reset can change it.
    ferrix_core::state::RESET_REASON.store(
        reset_reason(),
        Ordering::Relaxed,
    );

    peripherals
}

/// Read `POWER.RESETREAS` (0x4000_0400).
pub fn reset_reason() -> u32 {
    unsafe { core::ptr::read_volatile(0x4000_0400 as *const u32) }
}
