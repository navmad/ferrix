//! Fatal-fault LED indicator.
//!
//! These helpers drive a status LED with raw GPIO register writes and a
//! busy-wait delay, so they still work when the scheduler, the SoftDevice or
//! interrupts are wedged (panic, HardFault, SoftDevice assert). Boards keep
//! their own `#[panic_handler]` and `HardFault` and call [`FatalLed::blink_forever`]
//! with their pattern:
//!
//! - panic: 3 quick blinks, repeated forever
//! - HardFault: 6 quick blinks, repeated forever
//! - SoftDevice assert: 10 quick blinks, repeated forever

/// GPIO port of the indicator LED.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    P0,
    P1,
}

/// A status LED used for fatal-fault patterns.
#[derive(Clone, Copy, Debug)]
pub struct FatalLed {
    pub port: Port,
    /// Pin number within the port (0-31).
    pub pin: u8,
    /// `true` when driving the pin high lights the LED.
    pub active_high: bool,
}

const P0_BASE: u32 = 0x5000_0000;
const P1_BASE: u32 = 0x5000_0300;
const OUTSET: u32 = 0x508;
const OUTCLR: u32 = 0x50C;
const DIRSET: u32 = 0x518;
const PIN_CNF0: u32 = 0x700;

/// ~35 ms at 64 MHz.
const BLINK_CYCLES: u32 = 2_000_000;

impl FatalLed {
    fn base(&self) -> u32 {
        match self.port {
            Port::P0 => P0_BASE,
            Port::P1 => P1_BASE,
        }
    }

    fn write(&self, offset: u32, value: u32) {
        // SAFETY: fixed, always-mapped nRF52840 GPIO registers.
        unsafe { ((self.base() + offset) as *mut u32).write_volatile(value) }
    }

    fn mask(&self) -> u32 {
        1 << self.pin
    }

    /// Configure the pin as an output with the input buffer disconnected.
    pub fn init(&self) {
        // DIR = output, INPUT = disconnect.
        self.write(PIN_CNF0 + 4 * self.pin as u32, 0x0000_0003);
        self.write(DIRSET, self.mask());
    }

    pub fn on(&self) {
        self.write(if self.active_high { OUTSET } else { OUTCLR }, self.mask());
    }

    pub fn off(&self) {
        self.write(if self.active_high { OUTCLR } else { OUTSET }, self.mask());
    }

    #[inline(never)]
    fn delay() {
        cortex_m::asm::delay(BLINK_CYCLES);
    }

    /// One burst of `count` blinks followed by a gap.
    pub fn blink_burst(&self, count: u32) {
        self.init();
        for _ in 0..count {
            self.on();
            Self::delay();
            self.off();
            Self::delay();
        }
        for _ in 0..4 {
            Self::delay();
        }
    }

    /// Repeat a burst forever. Used for faults that cannot recover.
    pub fn blink_forever(&self, count: u32) -> ! {
        loop {
            self.blink_burst(count);
        }
    }
}
