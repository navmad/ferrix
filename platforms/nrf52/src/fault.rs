//! Safe fatal-fault LED indicator using nRF52 GPIO PAC registers directly.
//!
//! Uses busy-wait delays and direct GPIO register writes with no locks, no
//! timers, and no SoftDevice SVC calls, so it remains usable from panic,
//! `HardFault`, and SoftDevice fault-handler contexts.

use core::sync::atomic::{AtomicU8, Ordering};

use nrf_pac as pac;
use nrf_pac::gpio::vals;

/// GPIO pin used to blink fatal fault patterns.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FaultLed {
    pub port: u8,
    pub pin: u8,
}

impl FaultLed {
    pub const fn p0(pin: u8) -> Self {
        Self { port: 0, pin }
    }

    pub const fn p1(pin: u8) -> Self {
        Self { port: 1, pin }
    }

    const fn encode(self) -> u8 {
        if self.port == 0 && self.pin < 32 {
            self.pin
        } else if self.port == 1 && self.pin < 32 {
            32 + self.pin
        } else {
            NO_LED
        }
    }

    const fn decode(raw: u8) -> Option<Self> {
        if raw < 32 {
            Some(Self { port: 0, pin: raw })
        } else if raw < 64 {
            Some(Self {
                port: 1,
                pin: raw - 32,
            })
        } else {
            None
        }
    }
}

const NO_LED: u8 = 0xFF;

/// Board-registered fault LED pin (`0..32` = P0, `32..64` = P1, `0xFF` = none).
static FAULT_LED: AtomicU8 = AtomicU8::new(NO_LED);

/// Register the board's fatal-fault LED for SoftDevice fault callbacks.
pub fn set_fault_led(led: Option<FaultLed>) {
    let encoded = led.map_or(NO_LED, FaultLed::encode);
    FAULT_LED.store(encoded, Ordering::Relaxed);
}

fn gpio_port(port: u8) -> pac::gpio::Gpio {
    if port == 0 {
        pac::P0
    } else {
        pac::P1
    }
}

/// ~35 ms at 64 MHz. Used instead of a timer so it still works when the
/// scheduler, the SoftDevice, or interrupts are wedged.
#[inline(never)]
fn fatal_delay() {
    cortex_m::asm::delay(2_000_000);
}

fn led_init(led: FaultLed) {
    let port = gpio_port(led.port);
    let pin = led.pin as usize;
    port.pin_cnf(pin).write(|w| {
        w.set_dir(vals::Dir::OUTPUT);
        w.set_input(vals::Input::DISCONNECT);
    });
    port.dirset().write(|w| w.set_pin(pin, true));
}

fn led_on(led: FaultLed) {
    gpio_port(led.port)
        .outset()
        .write(|w| w.set_pin(led.pin as usize, true));
}

fn led_off(led: FaultLed) {
    gpio_port(led.port)
        .outclr()
        .write(|w| w.set_pin(led.pin as usize, true));
}

fn blink_burst(led: FaultLed, count: u32) {
    led_init(led);
    for _ in 0..count {
        led_on(led);
        fatal_delay();
        led_off(led);
        fatal_delay();
    }
    for _ in 0..4 {
        fatal_delay();
    }
}

/// Repeat a fatal blink burst forever on the given pin.
pub fn blink_pin_forever(led: FaultLed, count: u32) -> ! {
    loop {
        blink_burst(led, count);
    }
}

/// Repeat a fatal blink burst forever on the registered fault LED (or halt in
/// WFI if no LED was registered).
pub fn blink_forever(count: u32) -> ! {
    match FaultLed::decode(FAULT_LED.load(Ordering::Relaxed)) {
        Some(led) => blink_pin_forever(led, count),
        None => loop {
            cortex_m::asm::wfi();
        },
    }
}
