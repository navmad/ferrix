//! Reset and DFU entry points.

use defmt::info;

/// Reset the node through the ARM system control block.
pub fn system_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// Reboot into the Adafruit DFU bootloader.
///
/// `softdevice_active` selects the SoftDevice GPREGRET API; without it, write
/// the register directly (the SoftDevice owns it once enabled).
pub fn reboot_to_bootloader(softdevice_active: bool) -> ! {
    info!("Rebooting into Adafruit DFU Bootloader (GPREGRET=0x57)...");
    if softdevice_active {
        unsafe {
            let _ = nrf_softdevice::raw::sd_power_gpregret_clr(0, 0xFF);
            let _ = nrf_softdevice::raw::sd_power_gpregret_set(0, 0x57);
        }
    } else {
        unsafe {
            let gpregret = (0x4000_0000 + 0x51C) as *mut u32;
            gpregret.write_volatile(0x57);
        }
    }
    system_reset()
}
