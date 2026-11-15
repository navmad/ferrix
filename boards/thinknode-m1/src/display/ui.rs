//! Ferrix mesh-terminal UI for the 200x200 SSD1681 e-paper display.
//!
//! Eight pages share a mesh page rail, a top status strip and a two-action
//! footer. The bottom button navigates; the top button fires FK1 on a single
//! click and FK2 on a double click.

use core::fmt::Write as _;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use heapless::String;

use ferrix_core::companion::protocol::CompanionNode;
use ferrix_core::types::InboxItem;
use ferrix_runtime::app::{RadarSort, UiState};

use crate::display::{EpdFrameBuffer, EPD_WIDTH};

pub const PAGE_COUNT: u8 = 8;
pub const FIRMWARE_VERSION: &str = concat!(env!("CARGO_PKG_VERSION"), "-ferrix");
pub const PROTOCOL_VERSION: &str = "MeshCore v1.18";

const DISP_W: i32 = EPD_WIDTH as i32;
const SMALL_W: i32 = 6;
const TITLE_W: i32 = 9;
const LINE_H: i32 = 12;
const SEPARATOR_Y: i32 = 30;
const ACTION_SEP_Y: i32 = 162;

type SmallStyle = MonoTextStyle<'static, BinaryColor>;
type TitleStyle = MonoTextStyle<'static, BinaryColor>;

const SIN_COS: [(i16, i16); 32] = [
    (256, 0), (251, 50), (237, 98), (213, 142), (181, 181), (142, 213), (98, 237),
    (50, 251), (0, 256), (-50, 251), (-98, 237), (-142, 213), (-181, 181), (-213, 142),
    (-237, 98), (-251, 50), (-256, 0), (-251, -50), (-237, -98), (-213, -142),
    (-181, -181), (-142, -213), (-98, -237), (-50, -251), (0, -256), (50, -251),
    (98, -237), (142, -213), (181, -181), (213, -142), (237, -98), (251, -50),
];

/// Render the active page onto the framebuffer.
pub fn render_page(page: u8, node: &CompanionNode<'_>, ui: &UiState, fb: &mut EpdFrameBuffer) {
    fb.clear();
    let small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);

    let page = page.min(PAGE_COUNT - 1);
    draw_top_bar(node, fb, small, title);
    draw_mesh_rail(page, fb);
    draw_separator(fb, SEPARATOR_Y);

    match page {
        0 => page_mesh(fb, small, title),
        1 => page_inbox(node, ui, fb, small, title),
        2 => page_radar(node, ui, fb, small, title),
        3 => page_radio(fb, small, title),
        4 => page_signal(ui, node, fb, small, title),
        5 => page_position(node, fb, small, title),
        6 => page_offgrid(fb, small, title),
        _ => page_system(node, fb, small, title),
    }

    draw_action_bar(page, fb, small);
}

/// Top bar: callsign left, clock and battery right.
fn draw_top_bar(
    node: &CompanionNode<'_>,
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    let mut right: String<20> = String::new();
    if node.epoch_time > 1_700_000_000 {
        let secs = (node.epoch_time % 86_400) as u32;
        let _ = write!(right, "{:02}:{:02} ", secs / 3600, (secs % 3600) / 60);
    }
    let _ = write!(right, "{}%", node.battery_percentage);
    draw_right(fb, right.as_str(), 14, small);

    let right_x = DISP_W - 2 - text_width(right.as_str(), SMALL_W);
    let max_chars = ((right_x - 4).max(0) / TITLE_W) as usize;
    let mut name: String<32> = String::new();
    for c in node.callsign.chars().take(max_chars) {
        let _ = name.push(c);
    }
    if name.is_empty() {
        let _ = name.push_str(crate::NAME);
    }
    draw_text(fb, name.as_str(), 2, 14, title);
}

/// A linked-node page rail. The current page is a filled node.
fn draw_mesh_rail(page: u8, fb: &mut EpdFrameBuffer) {
    let total = PAGE_COUNT as i32;
    let spacing = 12;
    let start = (DISP_W - (total - 1) * spacing) / 2;
    let style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);

    let _ = Line::new(
        Point::new(start, 20),
        Point::new(start + (total - 1) * spacing, 20),
    )
    .into_styled(style)
    .draw(fb);

    for i in 0..total {
        let x = start + i * spacing;
        if i == page as i32 {
            let _ = Rectangle::new(Point::new(x - 2, 18), Size::new(5, 5))
                .into_styled(fill)
                .draw(fb);
        } else {
            let _ = Rectangle::new(Point::new(x, 20), Size::new(1, 1))
                .into_styled(fill)
                .draw(fb);
        }
    }
}

fn draw_separator(fb: &mut EpdFrameBuffer, y: i32) {
    let line = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let _ = Line::new(Point::new(0, y), Point::new(DISP_W - 1, y))
        .into_styled(line)
        .draw(fb);
}

fn page_mesh(
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    draw_text(fb, "MESH", 2, 50, title);
    let role = if ferrix_core::state::forwarding_enabled() {
        "repeater / forwarding"
    } else {
        "client"
    };
    draw_text(fb, "ONLINE", 2, 70, small);
    draw_text(fb, role, 2, 82, small);

    draw_mesh_glyph(fb, 100, 112, 24);

    let unread = ferrix_core::state::MESSAGE_LOG.lock(|m| m.borrow().len());
    let mut row: String<32> = String::new();
    let _ = write!(row, "unread {}", unread);
    draw_text(fb, row.as_str(), 2, 142, small);
    let ble = if ferrix_core::state::BLE_CONNECTED.load(core::sync::atomic::Ordering::Relaxed) {
        "BLE <ok>"
    } else {
        "BLE --"
    };
    draw_right(fb, ble, 142, small);

    draw_text(fb, uptime_label().as_str(), 2, 154, small);
}

fn page_inbox(
    node: &CompanionNode<'_>,
    ui: &UiState,
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    let count = ferrix_core::state::MESSAGE_LOG.lock(|m| m.borrow().len());
    draw_text(fb, "INBOX", 2, 50, title);
    let mut right: String<16> = String::new();
    let _ = write!(right, "{} NEW", count);
    draw_right(fb, right.as_str(), 50, small);

    if count == 0 {
        draw_centered(fb, "(no messages)", 100, small);
        return;
    }

    let cursor = ui.inbox_cursor.min(count - 1);
    let item = ferrix_core::state::MESSAGE_LOG.lock(|m| m.borrow().get(cursor).cloned());
    if let Some(item) = item {
        match item {
            InboxItem::Text(msg) => {
                let mut line: String<28> = String::new();
                let _ = write!(line, "From: {}", msg.sender.as_str());
                draw_text(fb, line.as_str(), 2, 72, small);
                if msg.timestamp > 1_700_000_000 {
                    draw_right(fb, clock_label(msg.timestamp).as_str(), 72, small);
                }
                draw_wrapped(fb, msg.text.as_str(), 2, 90, 31, 4, small);
            }
            InboxItem::ChannelData(data) => {
                let mut line: String<40> = String::new();
                let _ = write!(
                    line,
                    "DATA ch {} type {} ({} B)",
                    data.channel_idx,
                    data.data_type,
                    data.data.len()
                );
                draw_text(fb, line.as_str(), 2, 82, small);
                let mut snr: String<20> = String::new();
                let _ = write!(snr, "SNR {} dB", data.snr);
                draw_text(fb, snr.as_str(), 2, 104, small);
            }
        }
    }

    let mut footer: String<24> = String::new();
    let _ = write!(footer, "message {} of {}", cursor + 1, count);
    draw_text(fb, footer.as_str(), 2, 148, small);
    let _ = node;
}

fn page_radar(
    node: &CompanionNode<'_>,
    ui: &UiState,
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    let cx = 100;
    let cy = 104;
    let max_r = 48;

    draw_text(fb, "RADAR", 2, 50, title);

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
            let radius = 14 + metric * (max_r - 14) / 1000;
            let mut name: String<12> = String::new();
            for ch in ct.callsign.chars().take(11) {
                let _ = name.push(ch);
            }
            let _ = dots.push(RadarDot {
                angle,
                radius,
                name,
                labeled: false,
            });
        }
    });

    let ring = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    for r in [16, 32, 48] {
        let _ = Circle::new(Point::new(cx - r, cy - r), (r * 2) as u32)
            .into_styled(ring)
            .draw(fb);
    }
    let _ = Line::new(Point::new(cx - 4, cy), Point::new(cx + 4, cy))
        .into_styled(ring)
        .draw(fb);
    let _ = Line::new(Point::new(cx, cy - 4), Point::new(cx, cy + 4))
        .into_styled(ring)
        .draw(fb);
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    let _ = Rectangle::new(Point::new(cx - 1, cy - 1), Size::new(3, 3))
        .into_styled(fill)
        .draw(fb);

    if dots.is_empty() {
        draw_centered(fb, "(no contacts)", cy, small);
    }

    for dot in dots.iter() {
        let (cos, sin) = SIN_COS[dot.angle];
        let x = cx + (dot.radius * cos as i32) / 256;
        let y = cy + (dot.radius * sin as i32) / 256;
        let size = if dot.radius < 28 { 3 } else { 2 };
        let _ = Rectangle::new(Point::new(x - 1, y - 1), Size::new(size as u32, size as u32))
            .into_styled(fill)
            .draw(fb);
    }

    // Label the three nearest contacts when labels are enabled.
    let label_count = if ui.radar_labels { 3 } else { 0 };
    for _ in 0..label_count {
        let mut best: Option<(usize, i32)> = None;
        for (i, dot) in dots.iter().enumerate() {
            if dot.labeled {
                continue;
            }
            if best.map_or(true, |(_, r)| dot.radius < r) {
                best = Some((i, dot.radius));
            }
        }
        if let Some((idx, _)) = best {
            dots[idx].labeled = true;
            let dot = &dots[idx];
            let (cos, sin) = SIN_COS[dot.angle];
            let lx = cx + ((dot.radius + 5) * cos as i32) / 256;
            let ly = cy + ((dot.radius + 5) * sin as i32) / 256;
            let w = text_width(dot.name.as_str(), SMALL_W);
            let x = (lx - w / 2).clamp(2, DISP_W - 2 - w);
            let y = ly.clamp(40, 150);
            draw_text(fb, dot.name.as_str(), x, y, small);
        }
    }

    let mut status: String<24> = String::new();
    let _ = write!(
        status,
        "{} nodes  sort {}",
        dots.len(),
        match ui.radar_sort {
            RadarSort::Age => "AGE",
            RadarSort::Signal => "SIGNAL",
        }
    );
    draw_text(fb, status.as_str(), 2, 156, small);
}

fn page_radio(fb: &mut EpdFrameBuffer, small: SmallStyle, title: TitleStyle) {
    let (freq, bw, sf, cr, tx, duty) = ferrix_core::state::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (s.frequency_hz, s.bw_khz, s.sf, s.cr, s.tx_power_dbm, s.rx_duty_cycle)
    });

    draw_text(fb, "RADIO", 2, 50, title);
    let mut hero: String<24> = String::new();
    let _ = write!(
        hero,
        "{}.{:03}",
        freq / 1_000_000,
        (freq % 1_000_000 + 500) / 1000
    );
    draw_text(fb, hero.as_str(), 2, 78, title);
    draw_text(fb, "MHz", 2 + text_width(hero.as_str(), TITLE_W) + 3, 78, small);

    let mut row: String<40> = String::new();
    let _ = write!(row, "BW {}  SF{}  CR 4/{}", format_bw(bw).as_str(), sf, cr);
    draw_text(fb, row.as_str(), 2, 100, small);
    row.clear();
    let _ = write!(row, "TX {} dBm   RX {}", tx, if duty { "DUTY" } else { "CONT" });
    draw_text(fb, row.as_str(), 2, 114, small);
    row.clear();
    let af = ferrix_core::state::RADIO_SETTINGS
        .lock(|r| r.borrow().airtime_factor_tenths);
    let duty_x10 = 10_000 / (af as u32 + 10);
    let _ = write!(row, "airtime {}.{}%", duty_x10 / 10, duty_x10 % 10);
    draw_text(fb, row.as_str(), 2, 128, small);

    // Signal bars.
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    for i in 0..4 {
        let h = 5 + i * 4;
        let _ = Rectangle::new(Point::new(170 + i * 6, 140 - h), Size::new(4, h as u32))
            .into_styled(fill)
            .draw(fb);
    }
}

fn page_signal(
    ui: &UiState,
    _node: &CompanionNode<'_>,
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    draw_text(fb, "SIGNAL", 2, 50, title);
    let (rssi, snr) = ui.signal.last().unwrap_or((0, 0));
    let mut hero: String<20> = String::new();
    if ui.signal.len() == 0 {
        let _ = write!(hero, "-- dBm");
    } else {
        let _ = write!(hero, "{} dBm", rssi);
    }
    draw_text(fb, hero.as_str(), 2, 76, title);
    let mut snr_s: String<16> = String::new();
    let _ = write!(snr_s, "SNR {} dB", snr);
    draw_right(fb, snr_s.as_str(), 76, small);

    // RSSI sparkline.
    let hist = &ui.signal;
    if hist.len() > 0 {
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);
        let n = hist.len();
        for i in 0..n {
            if let Some((v, _)) = hist.at(i) {
                let x = 10 + (i as i32 * 178) / (n.saturating_sub(1).max(1) as i32);
                let norm = (v as i32 + 120).clamp(0, 80);
                let h = norm * 26 / 80;
                let y = 118 - h;
                let _ = Rectangle::new(Point::new(x, y), Size::new(2, h as u32))
                    .into_styled(fill)
                    .draw(fb);
            }
        }
    }
    draw_separator(fb, 124);

    let (rx, tx) = ferrix_core::state::RADIO_TELEMETRY.lock(|t| {
        let t = t.borrow();
        (t.rx_packets, t.tx_beacons)
    });
    let mut row: String<40> = String::new();
    let _ = write!(row, "rx {}   tx {}", rx, tx);
    draw_text(fb, row.as_str(), 2, 138, small);
    row.clear();
    let _ = write!(row, "air {} ms", ferrix_core::state::RADIO_TELEMETRY.lock(|t| t.borrow().rx_air_ms));
    draw_text(fb, row.as_str(), 2, 150, small);
}

fn page_position(
    node: &CompanionNode<'_>,
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    draw_text(fb, "POSITION", 2, 50, title);
    let fix = if node.gps_fix { "3D FIX" } else { "ACQUIRING" };
    draw_right(fb, fix, 50, small);

    let mut row: String<40> = String::new();
    let _ = write!(row, "Lat {:.5}", node.latitude);
    draw_text(fb, row.as_str(), 2, 72, small);
    row.clear();
    let _ = write!(row, "Lon {:.5}", node.longitude);
    draw_text(fb, row.as_str(), 2, 86, small);

    let power = if crate::gps::gps_enabled() { "ON" } else { "OFF" };
    let policy = ferrix_core::state::prefs().advert_loc_policy;
    row.clear();
    let _ = write!(row, "GPS {}", power);
    draw_text(fb, row.as_str(), 2, 106, small);
    row.clear();
    let _ = write!(row, "advert LOC {}", if policy != 0 { "ON" } else { "OFF" });
    draw_text(fb, row.as_str(), 2, 120, small);
    draw_text(fb, "sats --   age --", 2, 136, small);

    // Satellite glyph.
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let _ = Circle::new(Point::new(150, 88), 24)
        .into_styled(stroke)
        .draw(fb);
    let _ = Line::new(Point::new(162, 84), Point::new(162, 116))
        .into_styled(stroke)
        .draw(fb);
    let _ = Line::new(Point::new(146, 100), Point::new(178, 100))
        .into_styled(stroke)
        .draw(fb);
}

fn page_offgrid(fb: &mut EpdFrameBuffer, small: SmallStyle, title: TitleStyle) {
    let (active, client, freq) = ferrix_core::state::RADIO_SETTINGS
        .lock(|r| {
            let s = r.borrow();
            (
                s.client_repeat && ferrix_core::state::is_offgrid_freq_hz(s.frequency_hz),
                s.client_repeat,
                s.frequency_hz,
            )
        });

    draw_text(fb, "OFF-GRID", 2, 50, title);
    draw_right(fb, if active { "ON" } else { "OFF" }, 50, small);

    let mut row: String<40> = String::new();
    let _ = write!(
        row,
        "Channel {}.{:03}",
        freq / 1_000_000,
        (freq % 1_000_000 + 500) / 1000
    );
    draw_text(fb, row.as_str(), 2, 74, small);
    row.clear();
    let _ = write!(row, "client repeat {}", if client { "ON" } else { "OFF" });
    draw_text(fb, row.as_str(), 2, 90, small);
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let _ = Circle::new(Point::new(126, 92), 48)
        .into_styled(stroke)
        .draw(fb);
    draw_text(fb, "off-grid", 128, 120, small);
}

fn page_system(
    node: &CompanionNode<'_>,
    fb: &mut EpdFrameBuffer,
    small: SmallStyle,
    title: TitleStyle,
) {
    draw_text(fb, "SYSTEM", 2, 50, title);

    let mut row: String<40> = String::new();
    let _ = write!(row, "BATTERY {}%", node.battery_percentage);
    draw_text(fb, row.as_str(), 2, 72, small);
    draw_battery(fb, node.battery_percentage, 150, 64);

    draw_text(fb, uptime_label().as_str(), 2, 86, small);

    let (duty, path) = ferrix_core::state::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (s.rx_duty_cycle, s.path_hash_mode)
    });
    row.clear();
    let _ = write!(
        row,
        "LEDs {}   RX duty {}",
        if ferrix_core::state::leds_disabled() { "OFF" } else { "ON" },
        if duty { "ON" } else { "OFF" }
    );
    draw_text(fb, row.as_str(), 2, 100, small);
    row.clear();
    let _ = write!(row, "PATH {}B   hash {} byte/hop", path + 1, path + 1);
    draw_text(fb, row.as_str(), 2, 114, small);
    row.clear();
    let _ = write!(row, "FW {}", FIRMWARE_VERSION);
    draw_text(fb, row.as_str(), 2, 134, small);
    row.clear();
    let _ = write!(row, "BOARD {}", crate::NAME);
    draw_text(fb, row.as_str(), 2, 146, small);
}

fn draw_battery(fb: &mut EpdFrameBuffer, pct: u8, x: i32, y: i32) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    let _ = Rectangle::new(Point::new(x, y), Size::new(34, 14))
        .into_styled(stroke)
        .draw(fb);
    let _ = Rectangle::new(Point::new(x + 34, y + 4), Size::new(3, 6))
        .into_styled(fill)
        .draw(fb);
    let fill_w = ((pct.min(100) as i32) * 30) / 100;
    if fill_w > 0 {
        let _ = Rectangle::new(Point::new(x + 2, y + 2), Size::new(fill_w as u32, 10))
            .into_styled(fill)
            .draw(fb);
    }
}

fn draw_mesh_glyph(fb: &mut EpdFrameBuffer, cx: i32, cy: i32, r: i32) {
    let stroke = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);
    let nodes = [(cx, cy - r), (cx - r, cy), (cx + r, cy), (cx - r / 2, cy + r), (cx + r / 2, cy + r)];
    for (x, y) in nodes {
        let _ = Line::new(Point::new(cx, cy), Point::new(x, y))
            .into_styled(stroke)
            .draw(fb);
        let _ = Rectangle::new(Point::new(x - 2, y - 2), Size::new(4, 4))
            .into_styled(fill)
            .draw(fb);
    }
    let _ = Rectangle::new(Point::new(cx - 2, cy - 2), Size::new(5, 5))
        .into_styled(fill)
        .draw(fb);
}

fn draw_action_bar(page: u8, fb: &mut EpdFrameBuffer, small: SmallStyle) {
    draw_separator(fb, ACTION_SEP_Y);
    let (a1, a2) = actions(page);

    draw_text(fb, "FK1", 2, 174, small);
    draw_text(fb, a1, 24, 174, small);
    if !a2.is_empty() {
        draw_text(fb, "FK2", 2, 186, small);
        draw_text(fb, a2, 24, 186, small);
    }

    draw_text(fb, "TK next   TKx2 back", 2, 198, small);
    let mut page_no: String<12> = String::new();
    let _ = write!(page_no, "{:02}/{:02}", page + 1, PAGE_COUNT);
    draw_right(fb, page_no.as_str(), 198, small);
}

fn actions(page: u8) -> (&'static str, &'static str) {
    match page {
        0 => ("ZERO-HOP ADVERT", "FLOOD ADVERT"),
        1 => ("NEXT MESSAGE", "PREVIOUS MESSAGE"),
        2 => ("TOGGLE LABELS", "SORT AGE/SIGNAL"),
        3 => ("CYCLE TX POWER", "TOGGLE RX DUTY"),
        4 => ("RESET COUNTERS", "RESET SIGNAL"),
        5 => ("TOGGLE GPS", "TOGGLE LOCATION"),
        6 => ("TOGGLE OFF-GRID", "TOGGLE SOUND"),
        _ => ("TOGGLE LEDs", "PATH HASH 1B/2B/3B"),
    }
}

struct RadarDot {
    angle: usize,
    radius: i32,
    name: String<12>,
    labeled: bool,
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

fn draw_wrapped(
    fb: &mut EpdFrameBuffer,
    text: &str,
    x: i32,
    y: i32,
    max_chars: usize,
    max_lines: usize,
    style: SmallStyle,
) {
    let mut line: String<40> = String::new();
    let mut row = 0usize;
    for word in text.split_whitespace() {
        let add = if line.is_empty() { word.len() } else { word.len() + 1 };
        if line.len() + add > max_chars && !line.is_empty() {
            draw_text(fb, line.as_str(), x, y + row as i32 * LINE_H, style);
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
        draw_text(fb, line.as_str(), x, y + row as i32 * LINE_H, style);
    }
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

fn text_width(text: &str, char_w: i32) -> i32 {
    text.chars().count() as i32 * char_w
}

fn draw_text(fb: &mut EpdFrameBuffer, text: &str, x: i32, y: i32, style: SmallStyle) {
    let _ = Text::new(text, Point::new(x, y), style).draw(fb);
}

fn draw_right(fb: &mut EpdFrameBuffer, text: &str, y: i32, style: SmallStyle) {
    let x = DISP_W - 2 - text_width(text, SMALL_W);
    draw_text(fb, text, x, y, style);
}

fn draw_centered(fb: &mut EpdFrameBuffer, text: &str, y: i32, style: SmallStyle) {
    let x = ((DISP_W - text_width(text, SMALL_W)) / 2).max(0);
    draw_text(fb, text, x, y, style);
}

/// Boot splash: firmware, board, device, radio and protocol.
pub fn render_boot(node: &CompanionNode<'_>, fb: &mut EpdFrameBuffer) {
    use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
    let small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);

    fb.clear();

    let wordmark = "F E R R I X";
    let wx = ((DISP_W - text_width(wordmark, TITLE_W)) / 2).max(0);
    draw_text(fb, wordmark, wx, 38, title);
    draw_centered(fb, "mesh companion", 56, small);
    draw_separator(fb, 68);

    draw_text(fb, "FW", 2, 86, small);
    draw_text(fb, FIRMWARE_VERSION, 66, 86, small);
    draw_text(fb, "BOARD", 2, 100, small);
    draw_text(fb, crate::NAME, 66, 100, small);
    draw_text(fb, "DEVICE", 2, 114, small);
    let mut name: String<32> = String::new();
    let _ = name.push_str(node.callsign.as_str());
    if name.is_empty() {
        let _ = name.push_str("unnamed");
    }
    draw_text(fb, name.as_str(), 66, 114, small);
    draw_text(fb, "RADIO", 2, 128, small);
    draw_text(fb, "SX1262", 66, 128, small);
    draw_text(fb, "PROTO", 2, 142, small);
    draw_text(fb, PROTOCOL_VERSION, 66, 142, small);

    draw_mesh_glyph(fb, 100, 172, 14);
    draw_centered(fb, "starting mesh ....", 194, small);
}

/// Backward-compatible page render entry point.
pub fn render_dashboard(node: &CompanionNode<'_>, ui: &UiState, fb: &mut EpdFrameBuffer) {
    render_page(node.current_page, node, ui, fb);
}
