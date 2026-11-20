//! Reset and DFU entry points.

use defmt::info;
use nrf_pac as pac;

/// Reset the node through the ARM system control block.
pub fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// Adafruit nRF52 bootloader mode selected via `NRF_POWER->GPREGRET`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum BootloaderMode {
    /// `0x57`: USB UF2 mass-storage + CDC serial DFU.
    UsbUf2 = 0x57,
    /// `0xA8`: BLE OTA DFU (OTAFIX advertises `TNM1_DFU`).
    BleOta = 0xA8,
}

/// Clear any stale DFU magic in `NRF_POWER->GPREGRET` on boot before the
/// SoftDevice takes ownership of `POWER`.
pub fn clear_bootloader_magic() {
    pac::POWER.gpregret().write(|w| w.set_gpregret(0));
}

/// Reboot into the Adafruit DFU bootloader in USB UF2/Serial mode (`0x57`).
pub fn reboot_to_bootloader(softdevice_active: bool) -> ! {
    reboot_to_bootloader_mode(softdevice_active, BootloaderMode::UsbUf2)
}

/// Reboot into the Adafruit DFU bootloader with the requested [`BootloaderMode`].
///
/// `softdevice_active` selects the SoftDevice GPREGRET API; without it, write
/// the register directly (the SoftDevice owns it once enabled).
pub fn reboot_to_bootloader_mode(softdevice_active: bool, mode: BootloaderMode) -> ! {
    let magic = mode as u8;
    info!(
        "Rebooting into Adafruit DFU Bootloader (GPREGRET={=u8:#x})...",
        magic
    );
    if softdevice_active {
        crate::sd_ffi::power_gpregret_write(magic);
    } else {
        pac::POWER.gpregret().write(|w| w.set_gpregret(magic));
    }
    system_reset()
}
