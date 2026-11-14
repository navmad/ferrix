//! Board-facing re-exports for the platform USB transport.
//!
//! The USB CDC-ACM task and framing live in `platforms/nrf52`; the board keeps
//! its identity and persistence bindings here for the rest of the firmware.

pub use ferrix_nrf52::usb::{cdc_acm_task, init_usb, usb_task, UsbDriver};

pub use ferrix_core::airtime::bw_khz_to_hz;
pub use ferrix_core::command::{UsbCommand, USB_COMMAND_CHANNEL};
pub use ferrix_core::state::{
    current_private_key, current_public_key, default_channels, forwarding_enabled,
    get_device_time, init_leds_disabled, is_offgrid_freq_hz, is_offgrid_freq_khz, leds_disabled,
    nearest_offgrid_freq_hz, offgrid_active, offgrid_set, set_device_time, set_identity,
    DeviceClock, OffgridChange, AUTOADD_SETTINGS, CHANNELS, DEFAULT_PUBLIC_SECRET, DEVICE_CLOCK,
    DISCOVERED_CONTACTS, LEDS_DISABLED, MESSAGE_LOG, NODE_PRIVATE_KEY, NODE_PUBLIC_KEY,
    OFFGRID_FREQ_RANGES, RADIO_SETTINGS, RADIO_TELEMETRY,
};
pub use ferrix_core::types::{
    AutoAddSettings, ChannelData, DiscoveredContact, InboxItem, RadioSettings, RadioTelemetry,
    ReceivedChannelData, ReceivedMessage, MAX_CHANNELS, MAX_CONTACTS, MAX_MESSAGES,
};

/// Persist the user's LED preference.
pub fn set_leds_disabled(disabled: bool) {
    if LEDS_DISABLED.swap(disabled, core::sync::atomic::Ordering::Relaxed) != disabled {
        crate::storage::request_save_config();
    }
}
