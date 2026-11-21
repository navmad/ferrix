//! Golden-frame tests for the Wio Tracker L1 Pro OLED UI.
//!
//! Each scenario seeds the shared `ferrix_core::state` globals, renders into a
//! `ferrix_sh1106::Framebuffer` and compares a 128x64 ASCII dump
//! (`#` = lit pixel, `.` = dark) against `tests/golden/<name>.txt`.
//!
//! Regenerate after an intentional UI change and review the diff:
//!
//! ```text
//! BLESS=1 cargo test -p ferrix-wio-tracker-l1-pro-logic --target <host> --test snapshots
//! ```

use std::fmt::Write as _;
use std::path::PathBuf;
use std::sync::{Mutex, MutexGuard};

use embassy_time::{Duration, MockDriver};
use ferrix_core::companion::protocol::{CompanionHost, CompanionNode};
use ferrix_core::state;
use ferrix_core::types::{
    DiscoveredContact, InboxItem, RadioSettings, ReceivedChannelData, ReceivedMessage,
};
use ferrix_runtime::app::{RadarSort, UiState};
use ferrix_sh1106::{Framebuffer, HEIGHT, WIDTH};
use ferrix_wio_tracker_l1_pro_logic::ui::{
    render_boot, render_ota_dfu, render_page, PAGE_COUNT,
};

// The library is built without `cfg(test)` for integration tests, so the
// defmt/executor symbols that ferrix-core pulls in must be stubbed here too.
mod stubs {
    #[no_mangle]
    extern "C" fn _defmt_acquire() {}
    #[no_mangle]
    extern "C" fn _defmt_release() {}
    #[no_mangle]
    extern "C" fn _defmt_write(_bytes: *const u8, _len: usize) {}
    #[no_mangle]
    extern "C" fn _defmt_timestamp() {}
    #[no_mangle]
    extern "C" fn _defmt_panic() -> ! {
        panic!("defmt panic in host test")
    }
    #[no_mangle]
    extern "C" fn __pender(_context: *mut ()) {}
}

// The UI reads process-wide statics and the mock clock, so scenarios must not
// interleave.
static SERIAL: Mutex<()> = Mutex::new(());

fn serial() -> MutexGuard<'static, ()> {
    SERIAL.lock().unwrap_or_else(|e| e.into_inner())
}

#[derive(Clone, Copy)]
struct Host;

impl CompanionHost for Host {
    fn battery_millivolts(&self) -> u16 {
        4_000
    }
    fn set_callsign(&self, _name: &str) {}
    fn set_custom_priv_key(&self, _key: [u8; 64]) {}
    fn set_path_hash_mode(&self, _mode: u8) {}
    fn request_save_config(&self) {}
    fn request_radio_config(&self) {}
    fn request_factory_reset(&self) {}
    fn queue_consume_now(&self) {}
    fn advert_remember(&self, _pub_key: &[u8; 32], _payload: &[u8]) {}
    fn reboot(&self) -> ! {
        panic!("reboot in host test")
    }
}

static HOST: Host = Host;

// --- fixtures ---------------------------------------------------------------

const DEFAULT_RADIO: RadioSettings = RadioSettings {
    frequency_hz: 910_525_000,
    tx_power_dbm: 22,
    sf: 7,
    bw_khz: 62,
    cr: 5,
    is_repeater: false,
    client_repeat: false,
    offgrid_home_freq: 0,
    path_hash_mode: 0,
    rx_duty_cycle: false,
    airtime_factor_tenths: 90,
};

/// Back to a known power-on state: default radio, no messages or contacts,
/// uptime zero.
fn reset_state() {
    MockDriver::get().reset();
    state::RADIO_SETTINGS.lock(|r| *r.borrow_mut() = DEFAULT_RADIO);
    state::MESSAGE_LOG.lock(|m| m.borrow_mut().clear());
    state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
    state::BLE_CONNECTED.store(false, std::sync::atomic::Ordering::Relaxed);
    state::init_leds_disabled(false);
    state::set_buzzer_enabled(true);
    state::update_prefs(|p| *p = state::DEFAULT_PREFS);
    state::RADIO_TELEMETRY.lock(|t| {
        let mut t = t.borrow_mut();
        t.rx_packets = 0;
        t.tx_beacons = 0;
        t.rx_air_ms = 0;
    });
}

fn node(callsign: &str) -> CompanionNode<'static> {
    let mut n = CompanionNode::new(callsign, "Wio Tracker L1 Pro", 0, [7; 32], [9; 64], &HOST);
    n.battery_percentage = 0;
    n.battery_millivolts = 0;
    n
}

fn text(sender: &str, body: &str, timestamp: u32) -> InboxItem {
    let mut s = heapless::String::new();
    s.push_str(sender).unwrap();
    let mut t = heapless::String::new();
    t.push_str(body).unwrap();
    InboxItem::Text(ReceivedMessage {
        channel_idx: 0,
        path_len: 1,
        sender: s,
        sender_pub: [0; 32],
        text: t,
        timestamp,
        rssi: -90,
        snr: 6,
        is_direct: false,
        txt_type: 0,
        signed_prefix: [0; 4],
    })
}

fn contact(seed: u8, name: &str, last_seen: u32, rssi: i16) -> DiscoveredContact {
    let mut callsign = heapless::String::new();
    callsign.push_str(name).unwrap();
    DiscoveredContact {
        pub_key: [seed; 32],
        callsign,
        node_id: [seed; 4],
        last_seen_epoch: last_seen,
        rssi,
        snr: 5,
        latitude: 0,
        longitude: 0,
        flags: 0,
        contact_type: 1,
        out_path: [0; 64],
        out_path_len: 0xFF,
        last_advert_ts: 0,
        saved_hash: 0,
        sync_since: 0,
    }
}

const EPOCH: u64 = 1_800_000_000 + 13 * 3_600 + 7 * 60; // 13:07 UTC

/// A busy, fully populated node.
fn seed_full() -> (CompanionNode<'static>, UiState) {
    reset_state();
    MockDriver::get().advance(Duration::from_secs(3 * 3_600 + 25 * 60));

    state::RADIO_SETTINGS.lock(|r| {
        let mut r = r.borrow_mut();
        r.frequency_hz = 433_000_000;
        r.bw_khz = 125;
        r.sf = 10;
        r.cr = 6;
        r.tx_power_dbm = 14;
        r.rx_duty_cycle = true;
        r.client_repeat = true;
        r.path_hash_mode = 1;
        r.airtime_factor_tenths = 40;
    });
    state::BLE_CONNECTED.store(true, std::sync::atomic::Ordering::Relaxed);
    state::init_leds_disabled(true);
    state::set_buzzer_enabled(false);
    state::update_prefs(|p| p.advert_loc_policy = 1);
    state::RADIO_TELEMETRY.lock(|t| {
        let mut t = t.borrow_mut();
        t.rx_packets = 1_234;
        t.tx_beacons = 56;
        t.rx_air_ms = 98_765;
    });
    state::MESSAGE_LOG.lock(|m| {
        let mut m = m.borrow_mut();
        let _ = m.push(text("alice", "hello from the ridge, copy?", EPOCH as u32 - 120));
        let _ = m.push(text(
            "bob-the-builder-with-a-long-name",
            "supercalifragilisticexpialidocious wraps then clips at the edge of the panel",
            EPOCH as u32 - 60,
        ));
        let _ = m.push(text("carol", "ok", 0));
    });
    state::DISCOVERED_CONTACTS.lock(|c| {
        let mut c = c.borrow_mut();
        for i in 0..6u8 {
            let _ = c.push(contact(
                i + 1,
                &format!("node-{i}-abcdefghijk"),
                EPOCH as u32 - 100 * i as u32,
                -60 - 12 * i as i16,
            ));
        }
        // Never seen: must not appear on the radar.
        let _ = c.push(contact(99, "ghost", 0, -50));
    });

    let mut n = node("WioTracker-Alpha-Longname-01");
    n.epoch_time = EPOCH;
    n.battery_percentage = 50;
    n.battery_millivolts = 3_870;
    n.gps_fix = true;
    n.latitude = 47.60621;
    n.longitude = -122.33207;
    n.current_page = 0;

    let mut ui = UiState::default();
    ui.inbox_cursor = 1;
    ui.radar_sort = RadarSort::Signal;
    for (rssi, snr) in [(-110, -2), (-95, 3), (-80, 6), (-70, 8), (-100, 1), (-60, 9), (-90, 4)] {
        ui.signal.push(rssi, snr);
    }
    (n, ui)
}

/// Fresh boot: nothing received, no clock, no name.
fn seed_empty() -> (CompanionNode<'static>, UiState) {
    reset_state();
    (node(""), UiState::default())
}

// --- rendering + comparison -------------------------------------------------

fn ascii(fb: &Framebuffer) -> String {
    let bytes = fb.as_bytes();
    let mut out = String::with_capacity((WIDTH + 1) * HEIGHT);
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let on = bytes[(y / 8) * WIDTH + x] & (1 << (y % 8)) != 0;
            out.push(if on { '#' } else { '.' });
        }
        out.push('\n');
    }
    out
}

fn golden_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("golden")
        .join(format!("{name}.txt"))
}

fn check(name: &str, fb: &Framebuffer) {
    let got = ascii(fb);
    let path = golden_path(name);

    if std::env::var_os("BLESS").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, &got).unwrap();
        return;
    }

    let want = std::fs::read_to_string(&path).unwrap_or_else(|_| {
        panic!(
            "missing golden {}\nrun with BLESS=1 to create it",
            path.display()
        )
    });
    if got == want {
        return;
    }

    let mut report = String::new();
    let mut shown = 0;
    for (y, (g, w)) in got.lines().zip(want.lines()).enumerate() {
        if g != w {
            let _ = writeln!(report, "row {y:2} got : {g}\nrow {y:2} want: {w}");
            shown += 1;
            if shown == 4 {
                break;
            }
        }
    }
    panic!(
        "{name}: frame differs from {}\n{report}\nIf the change is intended, regenerate with BLESS=1",
        path.display()
    );
}

fn render_all_pages(prefix: &str, n: &mut CompanionNode<'_>, ui: &UiState, gps_on: bool) {
    for page in 0..PAGE_COUNT {
        n.current_page = page;
        let mut fb = Framebuffer::new();
        render_page(page, n, ui, gps_on, &mut fb);
        check(&format!("{prefix}_p{page}"), &fb);
    }
}

// --- golden scenarios -------------------------------------------------------

#[test]
fn golden_empty_node_all_pages() {
    let _g = serial();
    let (mut n, ui) = seed_empty();
    render_all_pages("empty", &mut n, &ui, false);
}

#[test]
fn golden_full_node_all_pages() {
    let _g = serial();
    let (mut n, ui) = seed_full();
    render_all_pages("full", &mut n, &ui, true);
}

#[test]
fn golden_position_gps_off_with_fix() {
    let _g = serial();
    let (n, ui) = seed_full();
    let mut fb = Framebuffer::new();
    render_page(5, &n, &ui, false, &mut fb);
    check("position_gps_off", &fb);
}

#[test]
fn golden_radio_extremes() {
    let _g = serial();
    let (n, ui) = seed_empty();
    state::RADIO_SETTINGS.lock(|r| {
        let mut r = r.borrow_mut();
        r.frequency_hz = 869_618_000;
        r.bw_khz = 7;
        r.sf = 12;
        r.cr = 8;
        r.tx_power_dbm = -9;
        r.airtime_factor_tenths = 0;
    });
    let mut fb = Framebuffer::new();
    render_page(3, &n, &ui, false, &mut fb);
    check("radio_extreme", &fb);
}

#[test]
fn golden_battery_low_and_full() {
    let _g = serial();
    let (mut n, ui) = seed_empty();
    n.battery_percentage = 5;
    n.battery_millivolts = 3_300;
    let mut fb = Framebuffer::new();
    render_page(8, &n, &ui, false, &mut fb);
    check("system_battery_low", &fb);

    n.battery_percentage = 100;
    n.battery_millivolts = 4_200;
    render_page(8, &n, &ui, false, &mut fb);
    check("system_battery_full", &fb);
}

#[test]
fn golden_inbox_channel_data() {
    let _g = serial();
    let (n, ui) = seed_empty();
    state::MESSAGE_LOG.lock(|m| {
        let mut data = heapless::Vec::new();
        data.extend_from_slice(&[1, 2, 3, 4]).unwrap();
        let _ = m.borrow_mut().push(InboxItem::ChannelData(ReceivedChannelData {
            channel_idx: 2,
            path_len: 0xFF,
            snr: -7,
            data_type: 0x1234,
            data,
        }));
    });
    let mut fb = Framebuffer::new();
    render_page(1, &n, &ui, false, &mut fb);
    check("inbox_channel_data", &fb);
}

#[test]
fn golden_inbox_cursor_is_clamped() {
    let _g = serial();
    let (n, mut ui) = seed_full();
    ui.inbox_cursor = 500;
    let mut fb = Framebuffer::new();
    render_page(1, &n, &ui, false, &mut fb);
    check("inbox_cursor_clamped", &fb);
}

#[test]
fn golden_boot_named_and_unnamed() {
    let _g = serial();
    let (n, _) = seed_empty();
    let mut fb = Framebuffer::new();
    render_boot(&n, &mut fb);
    check("boot_unnamed", &fb);

    let n = node("d1nk4n");
    render_boot(&n, &mut fb);
    check("boot_named", &fb);
}

#[test]
fn golden_ota_dfu_notice() {
    let _g = serial();
    let (n, _) = seed_empty();
    let mut fb = Framebuffer::new();
    render_ota_dfu(&n, &mut fb);
    check("ota_dfu", &fb);
}

// --- structural properties (no golden files) --------------------------------

fn lit(fb: &Framebuffer, x: usize, y: usize) -> bool {
    fb.as_bytes()[(y / 8) * WIDTH + x] & (1 << (y % 8)) != 0
}

fn row_full(fb: &Framebuffer, y: usize) -> bool {
    (0..WIDTH).all(|x| lit(fb, x, y))
}

fn lit_count(fb: &Framebuffer) -> usize {
    fb.as_bytes().iter().map(|b| b.count_ones() as usize).sum()
}

#[test]
fn every_page_has_header_and_footer_rules() {
    let _g = serial();
    for (label, (mut n, ui)) in [("empty", seed_empty()), ("full", seed_full())] {
        for page in 0..PAGE_COUNT {
            n.current_page = page;
            let mut fb = Framebuffer::new();
            render_page(page, &n, &ui, true, &mut fb);
            assert!(row_full(&fb, 11), "{label} p{page}: header rule missing");
            assert!(row_full(&fb, 53), "{label} p{page}: footer rule missing");
            // Footer hints live in y 55..63 and must be drawn.
            assert!(
                (0..WIDTH).any(|x| (55..63).any(|y| lit(&fb, x, y))),
                "{label} p{page}: footer text missing"
            );
        }
    }
}

#[test]
fn pages_are_all_different() {
    let _g = serial();
    let (mut n, ui) = seed_full();
    let mut frames = Vec::new();
    for page in 0..PAGE_COUNT {
        n.current_page = page;
        let mut fb = Framebuffer::new();
        render_page(page, &n, &ui, true, &mut fb);
        frames.push(fb.as_bytes().to_vec());
    }
    for a in 0..frames.len() {
        for b in a + 1..frames.len() {
            assert_ne!(frames[a], frames[b], "pages {a} and {b} render identically");
        }
    }
}

#[test]
fn out_of_range_page_renders_the_last_page() {
    let _g = serial();
    let (n, ui) = seed_full();
    let mut last = Framebuffer::new();
    render_page(PAGE_COUNT - 1, &n, &ui, true, &mut last);
    for page in [PAGE_COUNT, PAGE_COUNT + 1, 255] {
        let mut fb = Framebuffer::new();
        render_page(page, &n, &ui, true, &mut fb);
        assert_eq!(fb.as_bytes(), last.as_bytes(), "page {page}");
    }
}

#[test]
fn rendering_clears_the_previous_frame() {
    let _g = serial();
    let (n, ui) = seed_full();
    let mut fresh = Framebuffer::new();
    render_page(0, &n, &ui, true, &mut fresh);

    let mut dirty = Framebuffer::new();
    for y in 0..HEIGHT as i32 {
        for x in 0..WIDTH as i32 {
            dirty.set_pixel(x, y, true);
        }
    }
    render_page(0, &n, &ui, true, &mut dirty);
    assert_eq!(dirty.as_bytes(), fresh.as_bytes());
}

#[test]
fn overlong_text_never_collides_with_the_page_number() {
    let _g = serial();
    // 32-char callsign must be truncated so the page counter stays legible.
    let (mut n, ui) = seed_full();
    n.callsign.clear();
    n.callsign.push_str("ABCDEFGHIJKLMNOPQRSTUVWXYZ012345").unwrap();
    let mut with_name = Framebuffer::new();
    render_page(0, &n, &ui, true, &mut with_name);

    n.callsign.clear();
    n.callsign.push_str("X").unwrap();
    let mut short = Framebuffer::new();
    render_page(0, &n, &ui, true, &mut short);

    // The right-hand part of the top bar (page counter, clock, battery) must
    // be identical regardless of the callsign length.
    let right_start = WIDTH - 60;
    for y in 0..10 {
        for x in right_start..WIDTH {
            assert_eq!(
                lit(&with_name, x, y),
                lit(&short, x, y),
                "callsign overlaps the status area at ({x},{y})"
            );
        }
    }
}

#[test]
fn worst_case_state_renders_every_page_without_panicking() {
    let _g = serial();
    let (mut n, mut ui) = seed_full();
    state::MESSAGE_LOG.lock(|m| {
        let mut m = m.borrow_mut();
        m.clear();
        for i in 0..256u32 {
            let _ = m.push(text(
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345",
                &"w".repeat(150),
                1_800_000_000 + i,
            ));
        }
    });
    state::DISCOVERED_CONTACTS.lock(|c| {
        let mut c = c.borrow_mut();
        c.clear();
        for i in 0..350u32 {
            let _ = c.push(contact(
                (i % 250) as u8 + 1,
                "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345",
                1_700_000_001 + i,
                -140 + (i % 160) as i16,
            ));
        }
    });
    state::RADIO_TELEMETRY.lock(|t| {
        let mut t = t.borrow_mut();
        t.rx_packets = u32::MAX;
        t.tx_beacons = u32::MAX;
        t.rx_air_ms = u32::MAX;
    });
    MockDriver::get().advance(Duration::from_secs(400 * 86_400));
    n.callsign.clear();
    n.callsign.push_str("ABCDEFGHIJKLMNOPQRSTUVWXYZ012345").unwrap();
    n.epoch_time = u64::MAX;
    n.battery_percentage = 255;
    n.battery_millivolts = u16::MAX;
    n.latitude = -89.999999;
    n.longitude = 179.999999;
    ui.inbox_cursor = usize::MAX;
    for i in 0..100 {
        ui.signal.push(-150 + i, -20 + (i % 40) as i8);
    }

    for sort in [RadarSort::Age, RadarSort::Signal] {
        ui.radar_sort = sort;
        for labels in [true, false] {
            ui.radar_labels = labels;
            for page in 0..=PAGE_COUNT {
                let mut fb = Framebuffer::new();
                render_page(page, &n, &ui, true, &mut fb);
                assert!(lit_count(&fb) > 100, "page {page} is nearly blank");
            }
        }
    }
}

#[test]
fn radar_lists_only_contacts_that_were_seen() {
    let _g = serial();
    let (mut n, ui) = seed_full();
    n.current_page = 2;
    let mut with_ghost = Framebuffer::new();
    render_page(2, &n, &ui, true, &mut with_ghost);

    state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().retain(|ct| ct.callsign.as_str() != "ghost"));
    let mut without_ghost = Framebuffer::new();
    render_page(2, &n, &ui, true, &mut without_ghost);
    assert_eq!(with_ghost.as_bytes(), without_ghost.as_bytes());
}

#[test]
fn signal_page_reflects_history() {
    let _g = serial();
    let (n, mut ui) = seed_full();
    let mut with_history = Framebuffer::new();
    render_page(4, &n, &ui, true, &mut with_history);
    ui.signal.reset();
    let mut without = Framebuffer::new();
    render_page(4, &n, &ui, true, &mut without);
    assert_ne!(with_history.as_bytes(), without.as_bytes());
    assert!(lit_count(&with_history) > lit_count(&without));
}
