//! MeshCore Multi-Page UI Renderer for 1.54" 200x200 EPD
//!
//! The layout follows the reference companion dashboard: a top bar with the
//! node name on the left and clock/battery on the right, page-indicator dots,
//! a separator, then page content. There is no footer; the dots show where you
//! are.
//!
//! Pages:
//! 0. Messages / connection status
//! 1. Recently heard contacts
//! 2. LoRa radio and packet counters
//! 3. Traffic counters
//! 4. GPS status
//! 5. Off-grid (client repeat) status

use core::fmt::Write as _;

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;
use heapless::String;

use crate::boards::Board;
use crate::companion::protocol::CompanionNode;
use crate::display::epd::{EpdFrameBuffer, EPD_WIDTH};

/// Number of dashboard pages. Keep in sync with `main`'s page-turn modulo.
pub const PAGE_COUNT: u8 = 8;

const DISP_W: i32 = EPD_WIDTH as i32;
const SMALL_W: i32 = 6;
const TITLE_W: i32 = 9;
const LINE_H: i32 = 12;
const SEPARATOR_Y: i32 = 28;
/// Baseline of the first content row. Increase this to add space between the
/// separator line and every page's content.
const CONTENT_Y: i32 = 40;

type SmallStyle = MonoTextStyle<'static, BinaryColor>;
type TitleStyle = MonoTextStyle<'static, BinaryColor>;

/// Render the active page onto the framebuffer.
pub fn render_page(page: u8, node: &CompanionNode, fb: &mut EpdFrameBuffer) {
    fb.clear();

    let small = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);
    let line = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    let page = page.min(PAGE_COUNT - 1);
    draw_top_bar(node, fb, small, title);
    draw_page_dots(page, fb);
    let _ = Line::new(Point::new(0, SEPARATOR_Y), Point::new(DISP_W - 1, SEPARATOR_Y))
        .into_styled(line)
        .draw(fb);

    match page {
        0 => render_messages(node, fb, small),
        1 => render_recent(node, fb, small),
        2 => render_radio(fb, small),
        3 => render_traffic(fb, small),
        4 => render_gps(node, fb, small),
        5 => render_offgrid(fb, small),
        6 => render_leds(fb, small),
        _ => render_power(fb, small),
    }
    draw_action_bar(page, fb, small);
}

/// Bottom action bar: describes what the top button does on this page.
fn draw_action_bar(page: u8, fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let line = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
    let _ = Line::new(Point::new(0, 178), Point::new(DISP_W - 1, 178))
        .into_styled(line)
        .draw(fb);

    let action = match page {
        0 => "FK: flood advert",
        1 => "FK: zero-hop advert",
        2 => "FK: cycle TX power",
        3 => "FK: reset counters",
        4 => "FK: toggle GPS",
        5 => "FK: toggle offgrid",
        6 => "FK: toggle blue LED",
        _ => "FK: toggle RX duty",
    };
    draw_text(fb, action, 2, 192, small);
}

/// Top bar: node name on the left, clock + battery right-aligned.
fn draw_top_bar(node: &CompanionNode, fb: &mut EpdFrameBuffer, small: SmallStyle, title: TitleStyle) {
    // Right block first so the name can be clipped to the remaining space.
    let mut right: String<20> = String::new();
    if node.epoch_time > 1_700_000_000 {
        let secs = (node.epoch_time % 86_400) as u32;
        let _ = write!(right, "{:02}:{:02} ", secs / 3600, (secs % 3600) / 60);
    }
    let _ = write!(right, "{}%", node.battery_percentage);

    let right_x = DISP_W - 2 - text_width(right.as_str(), SMALL_W);
    draw_text(fb, right.as_str(), right_x, 14, small);

    // Node name — always the active callsign, never a page title.
    let available = (right_x - 4).max(0);
    let max_chars = (available / TITLE_W).max(0) as usize;
    let mut name: String<32> = String::new();
    for c in node.callsign.chars().take(max_chars) {
        let _ = name.push(c);
    }
    if name.is_empty() {
        let _ = name.push_str(crate::boards::CurrentBoard::NAME);
    }
    draw_text(fb, name.as_str(), 2, 14, title);
}

/// Page indicator: filled 3x3 block for the current page, single pixels for
/// the rest, centered under the top bar.
fn draw_page_dots(page: u8, fb: &mut EpdFrameBuffer) {
    let total = PAGE_COUNT as i32;
    let spacing = 10;
    let start_x = (DISP_W - (total - 1) * spacing) / 2;
    let fill = PrimitiveStyle::with_fill(BinaryColor::On);

    for i in 0..total {
        let x = start_x + i * spacing;
        if i == page as i32 {
            let _ = Rectangle::new(Point::new(x - 1, 19), Size::new(3, 3))
                .into_styled(fill)
                .draw(fb);
        } else {
            let _ = Rectangle::new(Point::new(x, 20), Size::new(1, 1))
                .into_styled(fill)
                .draw(fb);
        }
    }
}

fn render_messages(node: &CompanionNode, fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let ble_connected =
        crate::companion::ble::BLE_CONNECTED.load(core::sync::atomic::Ordering::Relaxed);
    let ble_status = crate::companion::ble::BLE_STATUS.load(core::sync::atomic::Ordering::Relaxed);

    let link = if ble_connected {
        "< Connected: BLE >"
    } else if ble_status <= 3 {
        // Not started yet, probing, enabled, or advertising.
        "Waiting for app..."
    } else {
        "BLE: off"
    };

    let mut buf: String<32> = String::new();
    let _ = write!(buf, "MSG: {}", node.unread_messages);
    draw_centered(fb, buf.as_str(), CONTENT_Y + 24, small);

    draw_centered(fb, link, CONTENT_Y + 46, small);

    buf.clear();
    let _ = write!(
        buf,
        "Offgrid: {}",
        if crate::companion::usb::offgrid_active() { "ON" } else { "OFF" }
    );
    draw_centered(fb, buf.as_str(), CONTENT_Y + 68, small);

    let up = embassy_time::Instant::now().as_secs();
    buf.clear();
    let _ = write!(
        buf,
        "Up: {}d {:02}h {:02}m",
        up / 86_400,
        (up % 86_400) / 3600,
        (up % 3600) / 60
    );
    draw_centered(fb, buf.as_str(), CONTENT_Y + 90, small);
}

fn render_recent(node: &CompanionNode, fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let mut recent: heapless::Vec<(u32, String<16>), 4> = heapless::Vec::new();

    crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
        for ct in c.borrow().iter() {
            if ct.last_seen_epoch == 0 {
                continue;
            }

            let mut name: String<16> = String::new();
            for ch in ct.callsign.chars().take(15) {
                let _ = name.push(ch);
            }

            let pos = recent
                .iter()
                .position(|(ts, _)| *ts < ct.last_seen_epoch)
                .unwrap_or(recent.len());

            if recent.len() == 4 {
                if let Some((last_ts, _)) = recent.last() {
                    if ct.last_seen_epoch <= *last_ts {
                        continue;
                    }
                }
                let _ = recent.pop();
            }
            let _ = recent.insert(pos.min(recent.len()), (ct.last_seen_epoch, name));
        }
    });

    if recent.is_empty() {
        draw_centered(fb, "(no contacts heard)", CONTENT_Y + 42, small);
        return;
    }

    for (i, (ts, name)) in recent.iter().enumerate() {
        let y = CONTENT_Y + 6 + (i as i32 * LINE_H);
        draw_text(fb, name.as_str(), 2, y, small);
        draw_right(fb, &age_label(*ts, node.epoch_time), y, small);
    }
}

fn render_radio(fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let (freq_hz, bw_khz, sf, cr, tx_power, rx_duty) =
        crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (
                s.frequency_hz,
                s.bw_khz,
                s.sf,
                s.cr,
                s.tx_power_dbm,
                s.rx_duty_cycle,
            )
        });
    let (rx, tx) = crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
        let t = t.borrow();
        (t.rx_packets, t.tx_beacons)
    });

    let mut buf: String<32> = String::new();
    let _ = write!(
        buf,
        "{}.{:03} BW{}",
        freq_hz / 1_000_000,
        (freq_hz % 1_000_000 + 500) / 1000,
        format_bw(bw_khz)
    );
    draw_text(fb, buf.as_str(), 2, CONTENT_Y, small);

    let preamble = if sf <= 8 { 32 } else { 16 };
    buf.clear();
    let _ = write!(buf, "SF{} CR{} SW12 P{}", sf, cr, preamble);
    draw_text(fb, buf.as_str(), 2, CONTENT_Y + LINE_H, small);

    buf.clear();
    let _ = write!(
        buf,
        "TX:{}dBm RX/{}",
        tx_power,
        if rx_duty { "DC" } else { "CONT" }
    );
    draw_text(fb, buf.as_str(), 2, CONTENT_Y + 2 * LINE_H, small);

    buf.clear();
    let _ = write!(buf, "NF--  R{} T{} E0", rx, tx);
    draw_text(fb, buf.as_str(), 2, CONTENT_Y + 3 * LINE_H, small);
}

fn render_traffic(fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let (rx, tx) = crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
        let t = t.borrow();
        (t.rx_packets, t.tx_beacons)
    });

    draw_centered(fb, "Traffic", CONTENT_Y + 18, small);

    let mut buf: String<32> = String::new();
    let _ = write!(buf, "RX: {}   TX: {}", rx, tx);
    draw_centered(fb, buf.as_str(), CONTENT_Y + 42, small);

    buf.clear();
    let _ = write!(buf, "ERR: 0");
    draw_centered(fb, buf.as_str(), CONTENT_Y + 58, small);
}

fn render_gps(node: &CompanionNode, fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let mut buf: String<32> = String::new();

    let power = crate::gps::gps_enabled();
    let power_text = if power { "Power: ON" } else { "Power: OFF" };

    if node.gps_fix {
        draw_centered(fb, "GPS: 3D fix", CONTENT_Y + 10, small);
        draw_text(fb, power_text, 2, CONTENT_Y + 26, small);
        let _ = write!(buf, "Lat: {:.5}", node.latitude);
        draw_text(fb, buf.as_str(), 2, CONTENT_Y + 42, small);
        buf.clear();
        let _ = write!(buf, "Lon: {:.5}", node.longitude);
        draw_text(fb, buf.as_str(), 2, CONTENT_Y + 56, small);
        draw_text(fb, "Sats: --   Age: --", 2, CONTENT_Y + 70, small);
    } else {
        draw_centered(fb, "GPS: acquiring", CONTENT_Y + 10, small);
        draw_text(fb, power_text, 2, CONTENT_Y + 26, small);
        draw_centered(fb, "No fix yet", CONTENT_Y + 42, small);
        let _ = write!(buf, "Lat: {:.5}", node.latitude);
        draw_text(fb, buf.as_str(), 2, CONTENT_Y + 58, small);
        buf.clear();
        let _ = write!(buf, "Lon: {:.5}", node.longitude);
        draw_text(fb, buf.as_str(), 2, CONTENT_Y + 72, small);
    }
}

fn render_leds(fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let disabled = crate::companion::usb::leds_disabled();
    draw_centered(fb, "Blue LED", CONTENT_Y + 10, small);
    draw_centered(fb, if disabled { "OFF" } else { "ON" }, CONTENT_Y + 30, small);
    draw_centered(fb, "FK: toggle", CONTENT_Y + 50, small);
}

fn render_power(fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let (duty, tx) = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (s.rx_duty_cycle, s.tx_power_dbm)
    });

    let mut buf: String<32> = String::new();
    draw_centered(fb, "Power", CONTENT_Y + 10, small);
    draw_centered(
        fb,
        if duty { "RX duty: ON" } else { "RX duty: OFF" },
        CONTENT_Y + 30,
        small,
    );
    let _ = write!(buf, "TX: {} dBm", tx);
    draw_centered(fb, buf.as_str(), CONTENT_Y + 50, small);
}

fn render_offgrid(fb: &mut EpdFrameBuffer, small: SmallStyle) {
    let (client_repeat, freq_hz) = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (s.client_repeat, s.frequency_hz)
    });
    let active = crate::companion::usb::offgrid_active();

    let mut buf: String<32> = String::new();
    draw_centered(
        fb,
        if active { "Offgrid: ON" } else { "Offgrid: OFF" },
        CONTENT_Y + 18,
        small,
    );

    buf.clear();
    let _ = write!(
        buf,
        "Channel: {}.{:03}",
        freq_hz / 1_000_000,
        (freq_hz % 1_000_000 + 500) / 1000
    );
    draw_centered(fb, buf.as_str(), CONTENT_Y + 42, small);

    buf.clear();
    let _ = write!(
        buf,
        "Client repeat: {}",
        if client_repeat { "on" } else { "off" }
    );
    draw_centered(fb, buf.as_str(), CONTENT_Y + 62, small);
}

fn age_label(last_seen: u32, now_epoch: u64) -> String<8> {
    let mut s: String<8> = String::new();
    if now_epoch < 1_700_000_000 || last_seen == 0 {
        let _ = s.push_str("--");
        return s;
    }
    let now = now_epoch as u32;
    let age = now.saturating_sub(last_seen);
    if age < 60 {
        let _ = write!(s, "{}s", age);
    } else if age < 3600 {
        let _ = write!(s, "{}m", age / 60);
    } else if age < 86_400 {
        let _ = write!(s, "{}h", age / 3600);
    } else {
        let _ = write!(s, "{}d", age / 86_400);
    }
    s
}

fn format_bw(bw_khz: u32) -> String<10> {
    let hz = crate::companion::usb::bw_khz_to_hz(bw_khz);
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

/// Backward compatibility helper for render_dashboard.
pub fn render_dashboard(node: &CompanionNode, fb: &mut EpdFrameBuffer) {
    render_page(node.current_page, node, fb);
}
