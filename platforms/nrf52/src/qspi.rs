//! External QSPI NOR flash support.
//!
//! Supported boards carry a 2 MB NOR flash on the nRF52840 QSPI peripheral:
//! a Macronix MX25R1635F on the ThinkNode M1 and a Puya P25Q16H on the Wio
//! Tracker L1. The ThinkNode's flash rail is switched, so bring-up is
//! deliberately late and retried: a cold boot after a UF2 update can beat the
//! rail.

use core::sync::atomic::Ordering;

use defmt::{error, info};
use embassy_nrf::gpio::AnyPin;
use embassy_nrf::peripherals;
use embassy_nrf::qspi::{self, Qspi};
use embassy_time::Timer;

use ferrix_core::state::{QSPI_ID, QSPI_STATUS};

/// Board wiring for the external QSPI flash. Boards pass their pins through
/// `gpio::Pin::degrade()` so one type serves every pinout.
pub struct QspiPins {
    pub qspi: peripherals::QSPI,
    pub sck: AnyPin,
    pub csn: AnyPin,
    pub io0: AnyPin,
    pub io1: AnyPin,
    pub io2: AnyPin,
    pub io3: AnyPin,
}

embassy_nrf::bind_interrupts!(struct QspiIrqs {
    QSPI => embassy_nrf::qspi::InterruptHandler<peripherals::QSPI>;
});

/// Total size of the external flash.
pub const FLASH_SIZE: u32 = 2 * 1024 * 1024;

/// Macronix MX25R1635F (ThinkNode M1).
pub const MX25R1635F_ID: [u8; 3] = [0xC2, 0x28, 0x15];
/// Puya P25Q16H (Wio Tracker L1).
pub const P25Q16H_ID: [u8; 3] = [0x85, 0x60, 0x15];

/// Create the QSPI driver in single-line mode at 16 MHz.
///
/// Single-line `FASTREAD`/`PP` avoids the flash's QPI-mode entry sequence and
/// matches the vendor's own configuration.
pub fn init(pins: QspiPins) -> Qspi<'static, peripherals::QSPI> {
    let mut config = qspi::Config::default();
    config.capacity = FLASH_SIZE;
    config.frequency = qspi::Frequency::M16;
    config.read_opcode = qspi::ReadOpcode::FASTREAD;
    config.write_opcode = qspi::WriteOpcode::PP;
    config.write_page_size = qspi::WritePageSize::_256BYTES;
    config.deep_power_down = Some(qspi::DeepPowerDownConfig {
        // Datasheet tDP (enter) is ~10 ms; the peripheral counts 16 us units.
        enter_time: 625,
        // tRES1 is well under 50 us; 64 us gives margin.
        exit_time: 4,
    });
    config.address_mode = qspi::AddressMode::_24BIT;
    config.spi_mode = qspi::SpiMode::MODE0;

    Qspi::new(
        pins.qspi,
        QspiIrqs,
        pins.sck,
        pins.csn,
        pins.io0,
        pins.io1,
        pins.io2,
        pins.io3,
        config,
    )
}

/// Read the JEDEC ID and verify that the flash answers.
///
/// Read-only: there is no erase/write probe. The stores exercise the write
/// path when they first save data, and a missing or unknown chip is reported
/// as unavailable rather than silently ignored.
pub async fn probe(
    flash: &mut Qspi<'static, peripherals::QSPI>,
    accepted_ids: &[[u8; 3]],
) -> bool {
    let mut id = [0u8; 3];
    let mut ready = false;

    for attempt in 0..10 {
        if attempt != 0 {
            Timer::after_millis(20).await;
        }
        if flash.custom_instruction(0x9F, &[], &mut id).await.is_ok() && accepted_ids.contains(&id) {
            ready = true;
            break;
        }
    }

    QSPI_ID.store(
        u32::from_be_bytes([0, id[0], id[1], id[2]]),
        Ordering::Relaxed,
    );

    if !ready {
        QSPI_STATUS.store(2, Ordering::Relaxed);
        error!(
            "QSPI: unexpected JEDEC ID {=u32:06x}",
            u32::from_be_bytes([0, id[0], id[1], id[2]])
        );
        return false;
    }

    QSPI_STATUS.store(1, Ordering::Relaxed);
    info!("QSPI: flash ready, JEDEC {=[u8]:x}", id);
    true
}
