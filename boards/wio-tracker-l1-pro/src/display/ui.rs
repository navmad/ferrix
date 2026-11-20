//! Ferrix mesh-terminal UI for the 128x64 SH1106 OLED.
//!
//! Nine pages with the same indices and meaning as the ThinkNode UI (the
//! runtime's page actions switch on those numbers). Layout:
//!
//! ```text
//! y  0..9   top bar: callsign, page n/9, clock + battery   (6x10)
//! y 11      separator
//! y 14..23  page hero line                                  (6x10)
//! y 26..51  three detail rows                               (5x8)
//! y 53      separator
//! y 55..62  action hints                                    (5x8)
//! ```
//!
//! Joystick left/right (and up/down) change page; the joystick press fires
//! the primary action ("OK") and the user button the secondary one ("BTN").

use core::fmt::Write as _;

use embedded_graphics::mono_font::ascii::{FONT_5X8, FONT_6X10};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use heapless::String;

use ferrix_core::companion::protocol::CompanionNode;
use ferrix_core::types::InboxItem;
use ferrix_runtime::app::{RadarSort, UiState};
use ferrix_sh1106::Framebuffer;

pub const PAGE_COUNT: u8 = 9;
pub const FIRMWARE_VERSION: &str = ferrix_core::version::FIRMWARE_VERSION;
pub const PROTOCOL_VERSION: &str = "MeshCore v1.18";

const W: i32 = ferrix_sh1106::WIDTH as i32;
const HERO_W: i32 = 6;
const SMALL_W: i32 = 5;
const HERO_Y: i32 = 14;
const ROWS: [i32; 3] = [26, 35, 44];
const LINE_H: i32 = 9;
const FOOTER_SEP_Y: i32 = 53;
const FOOTER_Y: i32 = 55;

type Style = MonoTextStyle<'static, BinaryColor>;

fn hero_style() -> Style {
    MonoTextStyle::new(&FONT_6X10, BinaryColor::On)
}

fn small_style() -> Style {
    MonoTextStyle::new(&FONT_5X8, BinaryColor::On)
}

const SIN_COS: [(i16, i16); 32] = [
    (256, 0), (251, 50), (237, 98), (213, 142), (181, 181), (142, 213), (98, 237),
    (50, 251), (0, 256), (-50, 251), (-98, 237), (-142, 213), (-181, 181), (-213, 142),
    (-237, 98), (-251, 50), (-256, 0), (-251, -50), (-237, -98), (-213, -142),
    (-181, -181), (-142, -213), (-98, -237), (-50, -251), (0, -256), (50, -251),
    (98, -237), (142, -213), (181, -181), (213, -142), (237, -98), (251, -50),
];

// --- text helpers -----------------------------------------------------------

fn text_width(text: &str, char_w: i32) -> i32 {
    text.chars().count() as i32 * char_w
}

fn draw(fb: &mut Framebuffer, text: &str, x: i32, y: i32, style: Style) {
    let _ = Text::with_baseline(text, Point::new(x, y), style, Baseline::Top).draw(fb);
}

fn draw_right(fb: &mut Framebuffer, text: &str, y: i32, style: Style, char_w: i32) {
    draw(fb, text, W - 2 - text_width(text, char_w), y, style);
}

fn draw_centered(fb: &mut Framebuffer, text: &str, y: i32, style: Style, char_w: i32) {
    draw(fb, text, ((W - text_width(text, char_w)) / 2).max(0), y, style);
}

fn hline(fb: &mut Framebuffer, y: i32) {
    let _ = Line::new(Point::new(0, y), Point::new(W - 1, y))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(fb);
}

fn fill_rect(fb: &mut Framebuffer, x: i32, y: i32, w: u32, h: u32) {
    let _ = Rectangle::new(Point::new(x, y), Size::new(w, h))
        .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
        .draw(fb);
}

fn stroke_rect(fb: &mut Framebuffer, x: i32, y: i32, w: u32, h: u32) {
    let _ = Rectangle::new(Point::new(x, y), Size::new(w, h))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(fb);
}

fn uptime_label() -> String<24> {
    let up = embassy_time::Instant::now().as_secs();
    let mut s: String<24> = String::new();
    let _ = write!(
        s,
        "up {}d {:02}h {:02}m",
        up / 86_400,
        (up % 86_400) / 3600,
        (up % 3600) / 60
    );
    s
}

fn clock_label(ts: u32) -> String<8> {
    let mut s: String<8> = String::new();
    let secs = ts % 86_400;
    let _ = write!(s, "{:02}:{:02}", secs / 3600, (secs % 3600) / 60);
    s
}

fn format_mhz(freq_hz: u32) -> String<12> {
    let mut s: String<12> = String::new();
    let _ = write!(
        s,
        "{}.{:03}",
        freq_hz / 1_000_000,
        (freq_hz % 1_000_000 + 500) / 1000
    );
    s
}

fn format_bw(bw_khz: u32) -> String<10> {
    let hz = ferrix_core::airtime::bw_khz_to_hz(bw_khz);
    let mut s: String<10> = String::new();
    if hz % 1000 == 0 {
        let _ = write!(s, "{}", hz / 1000);
    } else if hz % 100 == 0 {
        let _ = write!(s, "{}.{}", hz / 1000, (hz % 1000) / 100);
    } else {
        let _ = write!(s, "{}.{:02}", hz / 1000, (hz % 1000) / 10);
    }
    s
}

fn draw_wrapped(
    fb: &mut Framebuffer,
    text: &str,
    x: i32,
    y: i32,
    max_chars: usize,
    max_lines: usize,
    style: Style,
) {
    let mut line: String<40> = String::new();
    let mut row = 0usize;
    for word in text.split_whitespace() {
        let add = if line.is_empty() { word.len() } else { word.len() + 1 };
        if line.len() + add > max_chars && !line.is_empty() {
            draw(fb, line.as_str(), x, y + row as i32 * LINE_H, style);
            row += 1;
            if row >= max_lines {
                return;
            }
            line.clear();
        }
        if !line.is_empty() {
            let _ = line.push(' ');
        }
        for c in word.chars() {
            if line.len() >= max_chars {
                break;
            }
            let _ = line.push(c);
        }
    }
    if !line.is_empty() && row < max_lines {
        draw(fb, line.as_str(), x, y + row as i32 * LINE_H, style);
    }
}

// --- frame ------------------------------------------------------------------

/// Render the active page onto the framebuffer.
pub fn render_page(page: u8, node: &CompanionNode<'_>, ui: &UiState, fb: &mut Framebuffer) {
    fb.clear();
    let page = page.min(PAGE_COUNT - 1);
    draw_top_bar(page, node, fb);
    hline(fb, 11);

    match page {
        0 => page_mesh(fb),
        1 => page_inbox(ui, fb),
        2 => page_radar(node, ui, fb),
        3 => page_radio(fb),
        4 => page_signal(ui, fb),
        5 => page_position(node, fb),
        6 => page_offgrid(fb),
        7 => page_settings(fb),
        _ => page_system(node, fb),
    }

    hline(fb, FOOTER_SEP_Y);
    draw(fb, actions(page), 1, FOOTER_Y, small_style());
}

/// Callsign left, page number, clock and battery right.
fn draw_top_bar(page: u8, node: &CompanionNode<'_>, fb: &mut Framebuffer) {
    let hero = hero_style();
    let mut right: String<16> = String::new();
    if node.epoch_time > 1_700_000_000 {
        let secs = (node.epoch_time % 86_400) as u32;
        let _ = write!(right, "{:02}:{:02} ", secs / 3600, (secs % 3600) / 60);
    }
    let _ = write!(right, "{}%", node.battery_percentage);
    draw_right(fb, right.as_str(), 0, hero, HERO_W);

    let right_x = W - 2 - text_width(right.as_str(), HERO_W);
    let mut page_no: String<8> = String::new();
    let _ = write!(page_no, "{}/{}", page + 1, PAGE_COUNT);
    let page_x = right_x - 4 - text_width(page_no.as_str(), HERO_W);
    draw(fb, page_no.as_str(), page_x, 0, hero);

    let max_chars = ((page_x - 4).max(0) / HERO_W) as usize;
    let mut name: String<32> = String::new();
    for c in node.callsign.chars().take(max_chars) {
        let _ = name.push(c);
    }
    if name.is_empty() {
        for c in crate::NAME.chars().take(max_chars) {
            let _ = name.push(c);
        }
    }
    draw(fb, name.as_str(), 1, 0, hero);
}

fn actions(page: u8) -> &'static str {
    match page {
        0 => "OK zero-hop BTN flood",
        1 => "OK next BTN prev",
        2 => "OK labels BTN sort",
        3 => "OK txpwr BTN rxduty",
        4 => "OK rst cnt BTN rst sig",
        5 => "OK gps BTN location",
        6 => "OK off-grid",
        7 => "OK LEDs BTN sound",
        _ => "OK path BTN OTA DFU",
    }
}

// --- pages ------------------------------------------------------------------

fn page_mesh(fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    draw(fb, "MESH online", 1, HERO_Y, hero);
    let role = if ferrix_core::state::forwarding_enabled() {
        "FWD"
    } else {
        "CLIENT"
    };
    draw_right(fb, role, HERO_Y, hero, HERO_W);

    let unread = ferrix_core::state::MESSAGE_LOG.lock(|m| m.borrow().len());
    let peers = ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
    let mut row: String<32> = String::new();
    let _ = write!(row, "unread {}  peers {}", unread, peers);
    draw(fb, row.as_str(), 1, ROWS[0], small);
    let ble = if ferrix_core::state::BLE_CONNECTED.load(core::sync::atomic::Ordering::Relaxed) {
        "BLE connected"
    } else {
        "BLE --"
    };
    draw(fb, ble, 1, ROWS[1], small);
    draw(fb, uptime_label().as_str(), 1, ROWS[2], small);
}

fn page_inbox(ui: &UiState, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    let count = ferrix_core::state::MESSAGE_LOG.lock(|m| m.borrow().len());
    draw(fb, "INBOX", 1, HERO_Y, hero);

    if count == 0 {
        draw_centered(fb, "(no messages)", ROWS[1], small, SMALL_W);
        return;
    }

    let cursor = ui.inbox_cursor.min(count - 1);
    let mut pos: String<12> = String::new();
    let _ = write!(pos, "{}/{}", cursor + 1, count);
    draw_right(fb, pos.as_str(), HERO_Y, hero, HERO_W);

    let item = ferrix_core::state::MESSAGE_LOG.lock(|m| m.borrow().get(cursor).cloned());
    match item {
        Some(InboxItem::Text(msg)) => {
            let mut from: String<28> = String::new();
            let _ = write!(from, "{}", msg.sender.as_str());
            let name_chars = if msg.timestamp > 1_700_000_000 { 19 } else { 25 };
            let mut shown: String<28> = String::new();
            for c in from.chars().take(name_chars) {
                let _ = shown.push(c);
            }
            draw(fb, shown.as_str(), 1, ROWS[0], small);
            if msg.timestamp > 1_700_000_000 {
                draw_right(fb, clock_label(msg.timestamp).as_str(), ROWS[0], small, SMALL_W);
            }
            draw_wrapped(fb, msg.text.as_str(), 1, ROWS[1], 25, 2, small);
        }
        Some(InboxItem::ChannelData(data)) => {
            let mut line: String<40> = String::new();
            let _ = write!(
                line,
                "DATA ch{} t{} {}B",
                data.channel_idx,
                data.data_type,
                data.data.len()
            );
            draw(fb, line.as_str(), 1, ROWS[0], small);
            line.clear();
            let _ = write!(line, "SNR {} dB", data.snr);
            draw(fb, line.as_str(), 1, ROWS[1], small);
        }
        None => {}
    }
}

struct RadarDot {
    angle: usize,
    radius: i32,
    name: String<12>,
}

fn page_radar(node: &CompanionNode<'_>, ui: &UiState, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    let (cx, cy, max_r) = (96, 33, 18);

    draw(fb, "RADAR", 1, HERO_Y, hero);

    let mut dots: heapless::Vec<RadarDot, 20> = heapless::Vec::new();
    ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| {
        for ct in c.borrow().iter() {
            if ct.last_seen_epoch == 0 || dots.is_full() {
                continue;
            }
            let mut h = 2_166_136_261u32;
            for &b in ct.pub_key.iter() {
                h ^= b as u32;
                h = h.wrapping_mul(16_777_619);
            }
            let angle = (h as usize) % SIN_COS.len();
            let metric = match ui.radar_sort {
                RadarSort::Age => {
                    if node.epoch_time >= 1_700_000_000 {
                        let age = (node.epoch_time as u32)
                            .saturating_sub(ct.last_seen_epoch)
                            .min(3600);
                        age as i32 * 1000 / 3600
                    } else {
                        500
                    }
                }
                RadarSort::Signal => {
                    if ct.rssi != 0 {
                        ((-40 - ct.rssi).clamp(0, 80) as i32) * 1000 / 80
                    } else {
                        500
                    }
                }
            };
            let radius = 4 + metric * (max_r - 4) / 1000;
            let mut name: String<12> = String::new();
            for ch in ct.callsign.chars().take(11) {
                let _ = name.push(ch);
            }
            let _ = dots.push(RadarDot { angle, radius, name });
        }
    });

    let ring = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    for r in [9, 18] {
        let _ = Circle::new(Point::new(cx - r, cy - r), (r * 2) as u32)
            .into_styled(ring)
            .draw(fb);
    }
    fill_rect(fb, cx - 1, cy - 1, 3, 3);

    for dot in dots.iter() {
        let (cos, sin) = SIN_COS[dot.angle];
        let x = cx + (dot.radius * cos as i32) / 256;
        let y = cy + (dot.radius * sin as i32) / 256;
        fill_rect(fb, x - 1, y - 1, 2, 2);
    }

    let mut row: String<24> = String::new();
    let _ = write!(row, "{} nodes", dots.len());
    draw(fb, row.as_str(), 1, ROWS[0], small);
    draw(
        fb,
        match ui.radar_sort {
            RadarSort::Age => "sort AGE",
            RadarSort::Signal => "sort SIGNAL",
        },
        1,
        ROWS[1],
        small,
    );
    if ui.radar_labels {
        // Name of the nearest contact fits in the left column (12 chars).
        if let Some(near) = dots.iter().min_by_key(|d| d.radius) {
            draw(fb, near.name.as_str(), 1, ROWS[2], small);
        }
    } else {
        draw(fb, "labels off", 1, ROWS[2], small);
    }
}

fn page_radio(fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    let (freq, bw, sf, cr, tx, duty, af) = ferrix_core::state::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (
            s.frequency_hz,
            s.bw_khz,
            s.sf,
            s.cr,
            s.tx_power_dbm,
            s.rx_duty_cycle,
            s.airtime_factor_tenths,
        )
    });

    let mut line: String<24> = String::new();
    let _ = write!(line, "{} MHz", format_mhz(freq).as_str());
    draw(fb, line.as_str(), 1, HERO_Y, hero);

    line.clear();
    let _ = write!(line, "BW {} SF{} CR4/{}", format_bw(bw).as_str(), sf, cr);
    draw(fb, line.as_str(), 1, ROWS[0], small);
    line.clear();
    let _ = write!(line, "TX {} dBm  RX {}", tx, if duty { "DUTY" } else { "CONT" });
    draw(fb, line.as_str(), 1, ROWS[1], small);
    let duty_x10 = 10_000 / (af as u32 + 10);
    line.clear();
    let _ = write!(line, "airtime {}.{}%", duty_x10 / 10, duty_x10 % 10);
    draw(fb, line.as_str(), 1, ROWS[2], small);
}

fn page_signal(ui: &UiState, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    let (rssi, snr) = ui.signal.last().unwrap_or((0, 0));
    let mut line: String<20> = String::new();
    if ui.signal.len() == 0 {
        let _ = write!(line, "-- dBm");
    } else {
        let _ = write!(line, "{} dBm", rssi);
    }
    draw(fb, line.as_str(), 1, HERO_Y, hero);
    line.clear();
    let _ = write!(line, "SNR {}", snr);
    draw_right(fb, line.as_str(), HERO_Y, hero, HERO_W);

    // RSSI sparkline in y 26..42.
    let hist = &ui.signal;
    let n = hist.len();
    for i in 0..n {
        if let Some((v, _)) = hist.at(i) {
            let x = 2 + (i as i32 * 122) / (n.saturating_sub(1).max(1) as i32);
            let norm = (v as i32 + 120).clamp(0, 80);
            let h = (norm * 15 / 80).max(1);
            fill_rect(fb, x, 41 - h, 2, h as u32);
        }
    }

    let (rx, tx, air) = ferrix_core::state::RADIO_TELEMETRY.lock(|t| {
        let t = t.borrow();
        (t.rx_packets, t.tx_beacons, t.rx_air_ms)
    });
    line.clear();
    let _ = write!(line, "rx{} tx{} {}ms", rx, tx, air);
    draw(fb, line.as_str(), 1, ROWS[2], small);
}

fn page_position(node: &CompanionNode<'_>, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    draw(fb, "POSITION", 1, HERO_Y, hero);
    draw_right(
        fb,
        if node.gps_fix { "3D FIX" } else { "ACQ" },
        HERO_Y,
        hero,
        HERO_W,
    );

    let mut row: String<32> = String::new();
    let _ = write!(row, "Lat {:.5}", node.latitude);
    draw(fb, row.as_str(), 1, ROWS[0], small);
    row.clear();
    let _ = write!(row, "Lon {:.5}", node.longitude);
    draw(fb, row.as_str(), 1, ROWS[1], small);
    let policy = ferrix_core::state::prefs().advert_loc_policy;
    row.clear();
    let _ = write!(
        row,
        "GPS {}  advert LOC {}",
        if crate::gps::gps_enabled() { "ON" } else { "OFF" },
        if policy != 0 { "ON" } else { "OFF" }
    );
    draw(fb, row.as_str(), 1, ROWS[2], small);
}

fn page_offgrid(fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    let (active, client, freq) = ferrix_core::state::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (
            s.client_repeat && ferrix_core::state::is_offgrid_freq_hz(s.frequency_hz),
            s.client_repeat,
            s.frequency_hz,
        )
    });
    draw(fb, "OFF-GRID", 1, HERO_Y, hero);
    draw_right(fb, if active { "ON" } else { "OFF" }, HERO_Y, hero, HERO_W);

    let mut row: String<32> = String::new();
    let _ = write!(row, "Channel {} MHz", format_mhz(freq).as_str());
    draw(fb, row.as_str(), 1, ROWS[0], small);
    row.clear();
    let _ = write!(row, "client repeat {}", if client { "ON" } else { "OFF" });
    draw(fb, row.as_str(), 1, ROWS[1], small);
}

fn page_settings(fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    draw(fb, "SETTINGS", 1, HERO_Y, hero);
    let mut row: String<32> = String::new();
    let _ = write!(
        row,
        "LEDs   {}",
        if ferrix_core::state::leds_disabled() { "OFF" } else { "ON" }
    );
    draw(fb, row.as_str(), 1, ROWS[0], small);
    row.clear();
    let _ = write!(
        row,
        "SOUND  {}",
        if ferrix_core::state::buzzer_enabled() { "ON" } else { "OFF" }
    );
    draw(fb, row.as_str(), 1, ROWS[1], small);
}

fn page_system(node: &CompanionNode<'_>, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    draw(fb, "SYSTEM", 1, HERO_Y, hero);
    draw_battery(fb, node.battery_percentage, W - 2 - 22, HERO_Y);

    let mut row: String<40> = String::new();
    let _ = write!(
        row,
        "BAT {}% {}.{:02}V",
        node.battery_percentage,
        node.battery_millivolts / 1000,
        (node.battery_millivolts % 1000) / 10
    );
    draw(fb, row.as_str(), 1, ROWS[0], small);

    let path = ferrix_core::state::RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode);
    row.clear();
    let _ = write!(row, "FW {}  PATH {}B", FIRMWARE_VERSION, path + 1);
    draw(fb, row.as_str(), 1, ROWS[1], small);
    draw(fb, uptime_label().as_str(), 1, ROWS[2], small);
}

fn draw_battery(fb: &mut Framebuffer, pct: u8, x: i32, y: i32) {
    stroke_rect(fb, x, y, 20, 9);
    fill_rect(fb, x + 20, y + 3, 2, 3);
    let fill_w = ((pct.min(100) as i32) * 16) / 100;
    if fill_w > 0 {
        fill_rect(fb, x + 2, y + 2, fill_w as u32, 5);
    }
}

// --- boot / OTA / entry -----------------------------------------------------

/// Boot splash: firmware, board, device and protocol.
pub fn render_boot(node: &CompanionNode<'_>, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    fb.clear();
    draw_centered(fb, "F E R R I X", 2, hero, HERO_W);
    draw_centered(fb, "mesh companion", 14, small, SMALL_W);
    hline(fb, 24);

    draw(fb, "FW", 1, 28, small);
    draw(fb, FIRMWARE_VERSION, 36, 28, small);
    draw(fb, "BOARD", 1, 37, small);
    draw(fb, crate::NAME, 36, 37, small);
    draw(fb, "NODE", 1, 46, small);
    let mut name: String<32> = String::new();
    for c in node.callsign.chars().take(18) {
        let _ = name.push(c);
    }
    if name.is_empty() {
        let _ = name.push_str("unnamed");
    }
    draw(fb, name.as_str(), 36, 46, small);
    draw_centered(fb, "starting mesh ....", 55, small, SMALL_W);
}

/// Full-screen BLE OTA DFU notice shown before resetting into the bootloader.
pub fn render_ota_dfu(_node: &CompanionNode<'_>, fb: &mut Framebuffer) {
    let (hero, small) = (hero_style(), small_style());
    fb.clear();
    draw_centered(fb, "BLE OTA DFU", 2, hero, HERO_W);
    hline(fb, 14);
    draw_centered(fb, "Bootloader DFU mode", 22, small, SMALL_W);
    draw_centered(fb, "Flash .zip via nRF DFU", 33, small, SMALL_W);
    draw_centered(fb, "Reset button to cancel", 48, small, SMALL_W);
}

/// Page render entry point.
pub fn render_dashboard(node: &CompanionNode<'_>, ui: &UiState, fb: &mut Framebuffer) {
    render_page(node.current_page, node, ui, fb);
}
