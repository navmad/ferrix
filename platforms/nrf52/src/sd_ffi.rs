//! Quarantined Nordic S140 SoftDevice C FFI wrappers.
//!
//! This is the only module in `ferrix-nrf52` permitted to use `unsafe`. Every
//! call below invokes a raw S140 SVC entry point that `nrf-softdevice` does not
//! expose as a safe wrapper.

#![allow(unsafe_code)]

use nrf_softdevice::raw;

/// SoftDevice assertion callback passed to `sd_softdevice_enable`.
///
/// Runs in SoftDevice fault context; must not invoke SoftDevice SVCs or
/// critical-section locks.
unsafe extern "C" fn sd_fault_handler(id: u32, pc: u32, info: u32) {
    defmt::error!(
        "Softdevice fault: id={=u32:#x} pc={=u32:#x} info={=u32:#x}",
        id,
        pc,
        info
    );
    // A SoftDevice assert is fatal. Signal it with 10 blinks so the failure
    // can be identified without a debugger.
    crate::fault::blink_forever(10)
}

/// Probe SoftDevice enablement with the given LFCLK configuration without
/// touching `SCB->VTOR`. Returns the raw Nordic status code (`0` = success).
pub(crate) fn softdevice_probe_enable(clock: Option<&raw::nrf_clock_lf_cfg_t>) -> u32 {
    let p_clock = clock.map_or(core::ptr::null(), |cfg| cfg as *const _);
    // SAFETY: `p_clock` is either null or points to a valid `nrf_clock_lf_cfg_t`
    // for the duration of the synchronous SVC call; `sd_fault_handler` has
    // static lifetime and C ABI.
    unsafe { raw::sd_softdevice_enable(p_clock, Some(sd_fault_handler)) }
}

/// Disable a previously enabled SoftDevice so `nrf_softdevice::Softdevice::enable`
/// can re-enable and take ownership cleanly.
pub(crate) fn softdevice_disable() {
    // SAFETY: called immediately after `softdevice_probe_enable` succeeded and
    // before any SoftDevice tasks or borrows exist.
    unsafe {
        let _ = raw::sd_softdevice_disable();
    }
}

/// Enable the stage-1 (`REG1`) DC-DC converter while the SoftDevice owns `POWER`.
pub(crate) fn power_dcdc_enable() -> u32 {
    // SAFETY: called only after `Softdevice::enable` on boards with the external
    // REG1 inductor populated.
    unsafe {
        raw::sd_power_dcdc_mode_set(
            raw::NRF_POWER_DCDC_MODES_NRF_POWER_DCDC_ENABLE as u8,
        )
    }
}

/// Program the SoftDevice's static 6-digit pairing passkey.
///
/// The caller must keep `passkey` backed by stable storage while the SoftDevice
/// uses the option (our caller stores it in a static `Mutex<RefCell<[u8; 6]>>`).
pub(crate) fn ble_set_static_passkey(passkey: &[u8; 6]) -> u32 {
    let opt = raw::ble_opt_t {
        gap_opt: raw::ble_gap_opt_t {
            passkey: raw::ble_gap_opt_passkey_t {
                p_passkey: passkey.as_ptr(),
            },
        },
    };
    // SAFETY: `opt` holds a valid `BLE_GAP_OPT_PASSKEY` union variant pointing
    // to 6 ASCII digits, and the SoftDevice copies the passkey during the SVC.
    unsafe { raw::sd_ble_opt_set(raw::BLE_GAP_OPTS_BLE_GAP_OPT_PASSKEY, &opt) }
}

/// Write `GPREGRET` through the SoftDevice SVC API when the SoftDevice owns `POWER`.
pub(crate) fn power_gpregret_write(magic: u8) {
    // SAFETY: valid register index `0` and bitmasks; called right before reset
    // while the SoftDevice is active.
    unsafe {
        let _ = raw::sd_power_gpregret_clr(0, 0xFF);
        let _ = raw::sd_power_gpregret_set(0, magic as u32);
    }
}

/// Read the nRF52840 die temperature in `0.25 °C` units through the SoftDevice
/// SVC API when the SoftDevice owns `NRF_TEMP`.
pub(crate) fn temp_get_quarters() -> Option<i32> {
    let mut temp: i32 = 0;
    // SAFETY: `temp` is a valid aligned `i32` out-parameter for the synchronous SVC.
    let ret = unsafe { raw::sd_temp_get(&mut temp) };
    (ret == raw::NRF_SUCCESS).then_some(temp)
}
