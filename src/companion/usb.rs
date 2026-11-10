//! USB CDC-ACM Serial Interface and 1200-baud DFU Auto-Reset
//!
//! Provides:
//! 1. USB CDC-ACM communications for the MeshCore Companion CLI.
//! 2. 1200-baud touch detection: automatically reboots the nRF52840 into the
//!    DFU bootloader (`GPREGRET = 0x57`) when flashing over serial, eliminating
//!    the need for physical reset pinhole presses.

use embassy_nrf::peripherals::USBD;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;
use defmt::{info, debug};

pub type UsbDriver = Driver<'static, USBD, HardwareVbusDetect>;

static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State> = StaticCell::new();

/// Reboot the nRF52840 directly into the DFU Bootloader
pub fn reboot_to_bootloader() -> ! {
    info!("Rebooting into Adafruit DFU Bootloader (GPREGRET=0x57)...");
    unsafe {
        // NRF_POWER->GPREGRET (0x4000051C) = 0x57 (DFU Magic value)
        let gpregret = (0x4000_0000 + 0x51C) as *mut u32;
        gpregret.write_volatile(0x57);
    }
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
pub async fn cdc_acm_task(class: CdcAcmClass<'static, UsbDriver>) -> ! {
    let (_sender, mut receiver, control) = class.split_with_control();

    loop {
        receiver.wait_connection().await;
        info!("USB CDC-ACM Connected");

        loop {
            let changed_fut = control.control_changed();
            let mut rx_buf = [0u8; 64];
            let read_fut = receiver.read_packet(&mut rx_buf);

            match embassy_futures::select::select(changed_fut, read_fut).await {
                embassy_futures::select::Either::First(_) => {
                    let rate = receiver.line_coding().data_rate();
                    if rate == 1200 {
                        embassy_time::Timer::after_millis(30).await;
                        reboot_to_bootloader();
                    }
                }
                embassy_futures::select::Either::Second(read_res) => {
                    match read_res {
                        Ok(n) => {
                            let rate = receiver.line_coding().data_rate();
                            if rate == 1200 {
                                embassy_time::Timer::after_millis(30).await;
                                reboot_to_bootloader();
                            }
                            debug!("USB CDC received {} bytes", n);
                        }
                        Err(_) => break, // Host disconnected
                    }
                }
            }
        }
    }
}

/// Initialize USB CDC-ACM and return (UsbDevice, CdcAcmClass) ready to spawn
pub fn init_usb(
    driver: UsbDriver,
) -> (
    UsbDevice<'static, UsbDriver>,
    CdcAcmClass<'static, UsbDriver>,
) {
    // 0x239A is Adafruit VID, 0x8029 is Feather nRF52840 / ThinkNode M1
    let mut config = Config::new(0x239A, 0x8029);
    config.manufacturer = Some("MeshCore");
    config.product = Some("ThinkNode M1 Companion");
    // Omit serial number so macOS uses USB location ID (/dev/cu.usbmodem1101),
    // matching the bootloader port name for seamless 1200-baud touch flashing!
    config.serial_number = None;
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let msos_desc = MSOS_DESC.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let cdc_state = CDC_STATE.init(State::new());

    let mut builder = Builder::new(
        driver,
        config,
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, cdc_state, 64);
    let usb = builder.build();

    (usb, class)
}
