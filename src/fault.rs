//! Persistent panic / fault diagnostics.
//!
//! A panic on the target kills every task (USB, BLE, radio), so the only way
//! to learn what happened without a debug probe is to write the panic record
//! into RAM that survives a soft reset, reset once, and read it back over USB
//! on the next boot.
//!
//! The record lives in `.uninit`, which the linker marks `NOLOAD`: the C
//! runtime does not clear it, and a system reset does not clear SRAM on the
//! nRF52840. A power cycle does clear it, so the panic handler performs a
//! `SYSRESETREQ` instead of hanging.

use core::cell::UnsafeCell;
use core::fmt::Write as _;

pub const PANIC_MAGIC: u32 = 0x5041_4E43; // "PANC"

const MSG_MAX: usize = 112;
const FILE_MAX: usize = 64;

const RTT_TAIL_MAX: usize = 64;

#[repr(C)]
pub struct PanicRecord {
    pub magic: u32,
    pub line: u32,
    pub column: u32,
    pub pc: u32,
    pub msg_len: u8,
    pub file_len: u8,
    /// Number of consecutive panic-triggered resets. Guards against a boot loop.
    pub reset_budget: u8,
    pub _pad: u8,
    pub msg: [u8; MSG_MAX],
    pub file: [u8; FILE_MAX],
    /// defmt-rtt write cursor and the last encoded bytes at panic time. defmt
    /// logs the real panic message (e.g. "unwrap failed: ...") before calling
    /// core::panic!(), so this is the only place it can be recovered from.
    pub rtt_write: u32,
    pub rtt_size: u32,
    pub rtt_len: u8,
    pub _pad2: [u8; 3],
    pub rtt_tail: [u8; RTT_TAIL_MAX],
}

/// Layout of defmt-rtt's `_SEGGER_RTT` (see `defmt-rtt/src/lib.rs`).
#[repr(C)]
struct RttChannel {
    name: *const u8,
    buffer: *mut u8,
    size: usize,
    write: usize,
    read: usize,
    flags: usize,
}

#[repr(C)]
struct RttHeader {
    id: [u8; 16],
    max_up_channels: usize,
    max_down_channels: usize,
    up_channel: RttChannel,
}

extern "C" {
    static _SEGGER_RTT: RttHeader;
}

/// Snapshot the tail of the defmt RTT buffer, which holds the message logged
/// immediately before the panic.
fn capture_rtt(rec: &mut PanicRecord) {
    rec.rtt_len = 0;
    let hdr = unsafe { &_SEGGER_RTT };
    let size = hdr.up_channel.size;
    let write = hdr.up_channel.write;
    let buf = hdr.up_channel.buffer;
    rec.rtt_size = size as u32;
    rec.rtt_write = write as u32;
    if buf.is_null() || size == 0 {
        return;
    }
    let n = RTT_TAIL_MAX.min(size);
    for i in 0..n {
        let idx = (write + size - n + i) % size;
        // SAFETY: idx is within the RTT buffer.
        rec.rtt_tail[i] = unsafe { *buf.add(idx) };
    }
    rec.rtt_len = n as u8;
}

impl PanicRecord {
    const fn empty() -> Self {
        Self {
            magic: 0,
            line: 0,
            column: 0,
            pc: 0,
            msg_len: 0,
            file_len: 0,
            reset_budget: 0,
            _pad: 0,
            msg: [0; MSG_MAX],
            file: [0; FILE_MAX],
            rtt_write: 0,
            rtt_size: 0,
            rtt_len: 0,
            _pad2: [0; 3],
            rtt_tail: [0; RTT_TAIL_MAX],
        }
    }
}

struct RecordCell(UnsafeCell<PanicRecord>);
// SAFETY: written only by the panic / HardFault handler, which stops normal
// execution; read only from the boot path / CLI.
unsafe impl Sync for RecordCell {}

#[link_section = ".uninit.panic_record"]
static PANIC_RECORD: RecordCell = RecordCell(UnsafeCell::new(PanicRecord::empty()));

fn record_mut() -> &'static mut PanicRecord {
    // SAFETY: see `RecordCell`.
    unsafe { &mut *PANIC_RECORD.0.get() }
}

fn record_ref() -> &'static PanicRecord {
    // SAFETY: see `RecordCell`.
    unsafe { &*PANIC_RECORD.0.get() }
}

struct BufWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl core::fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let space = self.buf.len().saturating_sub(self.len);
        let n = s.len().min(space);
        self.buf[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

/// Copy a `PanicInfo` into the persistent record. Never logs and never takes a
/// lock, so it is safe to call from a panic that happened inside a critical
/// section.
pub fn record_panic(info: &core::panic::PanicInfo) {
    let rec = record_mut();
    let previous_budget = if rec.magic == PANIC_MAGIC {
        rec.reset_budget
    } else {
        0
    };

    capture_rtt(rec);
    rec.magic = 0; // in-progress marker
    rec.line = 0;
    rec.column = 0;
    rec.pc = 0;
    rec.msg_len = 0;
    rec.file_len = 0;

    if let Some(loc) = info.location() {
        rec.line = loc.line();
        rec.column = loc.column();
        let f = loc.file().as_bytes();
        let n = f.len().min(FILE_MAX);
        rec.file[..n].copy_from_slice(&f[..n]);
        rec.file_len = n as u8;
    }

    let mut w = BufWriter {
        buf: &mut rec.msg,
        len: 0,
    };
    let _ = write!(&mut w, "{}", info.message());
    rec.msg_len = w.len as u8;

    rec.reset_budget = previous_budget.saturating_add(1);
    rec.magic = PANIC_MAGIC;
}

/// Record a HardFault, including the faulting PC.
pub fn record_hardfault(pc: u32, lr: u32) {
    let rec = record_mut();
    let previous_budget = if rec.magic == PANIC_MAGIC {
        rec.reset_budget
    } else {
        0
    };

    capture_rtt(rec);
    rec.magic = 0;
    rec.line = 0;
    rec.column = 0;
    rec.pc = pc;
    rec.msg_len = 0;
    rec.file_len = 0;
    let msg = b"HardFault";
    rec.msg[..msg.len()].copy_from_slice(msg);
    rec.msg_len = msg.len() as u8;
    let file = b"hardfault";
    rec.file[..file.len()].copy_from_slice(file);
    rec.file_len = file.len() as u8;

    let mut w = BufWriter {
        buf: &mut rec.msg[msg.len()..],
        len: 0,
    };
    let _ = write!(&mut w, " pc={:#010x} lr={:#010x}", pc, lr);
    rec.msg_len = (msg.len() + w.len).min(MSG_MAX) as u8;

    rec.reset_budget = previous_budget.saturating_add(1);
    rec.magic = PANIC_MAGIC;
}

/// True once the panic-triggered reset has repeated too many times, so a
/// persistent boot-time panic does not become an endless reboot loop.
pub fn reset_budget_exhausted() -> bool {
    record_ref().reset_budget >= 5
}

/// Report whether a panic record is available.
pub fn has_record() -> bool {
    record_ref().magic == PANIC_MAGIC
}

pub struct PanicSnapshot {
    pub line: u32,
    pub column: u32,
    pub pc: u32,
    pub msg: heapless::String<MSG_MAX>,
    pub file: heapless::String<FILE_MAX>,
    pub rtt_write: u32,
    pub rtt_size: u32,
    pub rtt_len: u8,
    pub rtt_tail: [u8; RTT_TAIL_MAX],
}

/// Read the record without clearing it.
pub fn peek() -> Option<PanicSnapshot> {
    let rec = record_ref();
    if rec.magic != PANIC_MAGIC {
        return None;
    }
    let mut msg = heapless::String::new();
    let n = (rec.msg_len as usize).min(MSG_MAX);
    let _ = msg.push_str(core::str::from_utf8(&rec.msg[..n]).unwrap_or("<invalid utf8>"));
    let mut file = heapless::String::new();
    let n = (rec.file_len as usize).min(FILE_MAX);
    let _ = file.push_str(core::str::from_utf8(&rec.file[..n]).unwrap_or("<invalid utf8>"));
    Some(PanicSnapshot {
        line: rec.line,
        column: rec.column,
        pc: rec.pc,
        msg,
        file,
        rtt_write: rec.rtt_write,
        rtt_size: rec.rtt_size,
        rtt_len: rec.rtt_len,
        rtt_tail: rec.rtt_tail,
    })
}

/// Clear the record so it is not reported again.
pub fn clear() {
    record_mut().magic = 0;
}

/// Call once the node has booted and stayed healthy for a while, so a later
/// unrelated panic gets a fresh reset budget.
pub fn clear_reset_budget() {
    let rec = record_mut();
    if rec.magic == PANIC_MAGIC {
        rec.reset_budget = 0;
    }
}

/// Reset the core so the `.uninit` record survives into the next boot.
pub fn reset_now() -> ! {
    // Disable interrupts so no SoftDevice event can run during the reset.
    cortex_m::interrupt::disable();
    cortex_m::asm::dsb();
    // AIRCR: VECTKEY=0x05FA, SYSRESETREQ=1.
    unsafe {
        (0xE000_ED0C as *mut u32).write_volatile(0x05FA_0004);
    }
    loop {
        cortex_m::asm::dsb();
    }
}
