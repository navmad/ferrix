//! MeshCore Multi-Page UI Renderer for 1.54" 200x200 EPD
//!
//! Provides 3 distinct navigational pages:
//! 0. Main Companion Dashboard
//! 1. LoRa Radio & PNW Mesh Stats
//! 2. GNSS / GPS & Hardware Status

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;

use crate::companion::protocol::CompanionNode;
use crate::display::epd::EpdFrameBuffer;

/// Render the active page onto the framebuffer
pub fn render_page(page: u8, node: &CompanionNode, fb: &mut EpdFrameBuffer) {
    fb.clear();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title_style = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);
    let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    // Common Header: Title & Battery %
    let header_title = match page {
        1 => "LoRa Radio PNW",
        2 => "GNSS / GPS",
        _ => node.callsign.as_str(),
    };
    let _ = Text::new(header_title, Point::new(4, 14), title_style).draw(fb);

    let mut bat_str: heapless::String<12> = heapless::String::new();
    let _ = core::fmt::write(&mut bat_str, format_args!("{}%", node.battery_percentage));
    let _ = Text::new(&bat_str, Point::new(165, 14), text_style).draw(fb);

    // Header divider line
    let _ = Line::new(Point::new(0, 18), Point::new(199, 18))
        .into_styled(line_style)
        .draw(fb);

    match page {
        0 => render_page_main(node, fb, text_style, title_style, line_style),
        1 => render_page_radio(node, fb, text_style, title_style, line_style),
        _ => render_page_gps(node, fb, text_style, title_style, line_style),
    }

    // Common Footer
    let _ = Line::new(Point::new(0, 182), Point::new(199, 182))
        .into_styled(line_style)
        .draw(fb);

    let footer_text = match page {
        0 => "[1/3] Main | TK: Next",
        1 => "[2/3] Radio | TK: Next",
        _ => "[3/3] GPS | TK: Next",
    };
    let _ = Text::new(footer_text, Point::new(4, 194), text_style).draw(fb);
}

fn render_page_main(
    node: &CompanionNode,
    fb: &mut EpdFrameBuffer,
    text_style: MonoTextStyle<BinaryColor>,
    title_style: MonoTextStyle<BinaryColor>,
    line_style: PrimitiveStyle<BinaryColor>,
) {
    let _ = Text::new("Link: USB / BLE NUS", Point::new(4, 30), text_style).draw(fb);
    let _ = Text::new("Radio: 910.525 MHz", Point::new(4, 42), text_style).draw(fb);

    // Divider
    let _ = Line::new(Point::new(0, 48), Point::new(199, 48))
        .into_styled(line_style)
        .draw(fb);

    // Message / Companion Status Box
    let _ = Rectangle::new(Point::new(4, 54), Size::new(192, 122))
        .into_styled(line_style)
        .draw(fb);

    let _ = Text::new("MeshCore Node", Point::new(12, 70), title_style).draw(fb);
    let _ = Text::new("Status: Online & Ready", Point::new(12, 88), text_style).draw(fb);

    let mut unread_str: heapless::String<32> = heapless::String::new();
    let _ = core::fmt::write(&mut unread_str, format_args!("Unread: {} messages", node.unread_messages));
    let _ = Text::new(&unread_str, Point::new(12, 106), text_style).draw(fb);

    let mut tx_str: heapless::String<32> = heapless::String::new();
    let _ = core::fmt::write(&mut tx_str, format_args!("TX Beacons: {} sent", node.tx_beacons));
    let _ = Text::new(&tx_str, Point::new(12, 122), text_style).draw(fb);

    let mut rx_str: heapless::String<32> = heapless::String::new();
    let _ = core::fmt::write(&mut rx_str, format_args!("RX Packets: {} rcvd", node.rx_packets));
    let _ = Text::new(&rx_str, Point::new(12, 138), text_style).draw(fb);

    let _ = Text::new("FK: Beacon | TK: Page", Point::new(12, 160), text_style).draw(fb);
}

fn render_page_radio(
    node: &CompanionNode,
    fb: &mut EpdFrameBuffer,
    text_style: MonoTextStyle<BinaryColor>,
    title_style: MonoTextStyle<BinaryColor>,
    line_style: PrimitiveStyle<BinaryColor>,
) {
    let _ = Text::new("Profile: US PNW Preset", Point::new(4, 30), text_style).draw(fb);
    let _ = Text::new("Freq: 910.525 MHz", Point::new(4, 42), text_style).draw(fb);

    let _ = Line::new(Point::new(0, 48), Point::new(199, 48))
        .into_styled(line_style)
        .draw(fb);

    let _ = Rectangle::new(Point::new(4, 54), Size::new(192, 122))
        .into_styled(line_style)
        .draw(fb);

    let _ = Text::new("SX1262 Transceiver", Point::new(12, 70), title_style).draw(fb);
    let _ = Text::new("Modulation: LoRa SF7", Point::new(12, 86), text_style).draw(fb);
    let _ = Text::new("Bandwidth: 62.5 kHz", Point::new(12, 98), text_style).draw(fb);
    let _ = Text::new("Coding Rate: 4/5 (CR=1)", Point::new(12, 110), text_style).draw(fb);
    let _ = Text::new("TX Power: +22 dBm", Point::new(12, 122), text_style).draw(fb);

    let mut stats_str: heapless::String<32> = heapless::String::new();
    let _ = core::fmt::write(&mut stats_str, format_args!("TX: {} | RX: {}", node.tx_beacons, node.rx_packets));
    let _ = Text::new(&stats_str, Point::new(12, 140), text_style).draw(fb);

    let mut signal_str: heapless::String<32> = heapless::String::new();
    if node.last_rssi != 0 {
        let _ = core::fmt::write(&mut signal_str, format_args!("RSSI: {} | SNR: {}", node.last_rssi, node.last_snr));
    } else {
        let _ = core::fmt::write(&mut signal_str, format_args!("RSSI: None (Listening)"));
    }
    let _ = Text::new(&signal_str, Point::new(12, 154), text_style).draw(fb);
}

fn render_page_gps(
    node: &CompanionNode,
    fb: &mut EpdFrameBuffer,
    text_style: MonoTextStyle<BinaryColor>,
    title_style: MonoTextStyle<BinaryColor>,
    line_style: PrimitiveStyle<BinaryColor>,
) {
    let _ = Text::new("Receiver: Quectel L76K", Point::new(4, 30), text_style).draw(fb);
    let _ = Text::new("Baud: 9600 8N1 (UART0)", Point::new(4, 42), text_style).draw(fb);

    let _ = Line::new(Point::new(0, 48), Point::new(199, 48))
        .into_styled(line_style)
        .draw(fb);

    let _ = Rectangle::new(Point::new(4, 54), Size::new(192, 122))
        .into_styled(line_style)
        .draw(fb);

    let _ = Text::new("Position Telemetry", Point::new(12, 70), title_style).draw(fb);

    if node.gps_fix {
        let mut lat_str: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::write(&mut lat_str, format_args!("Lat: {:.5}", node.latitude));
        let _ = Text::new(&lat_str, Point::new(12, 90), text_style).draw(fb);

        let mut lon_str: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::write(&mut lon_str, format_args!("Lon: {:.5}", node.longitude));
        let _ = Text::new(&lon_str, Point::new(12, 104), text_style).draw(fb);
    } else {
        let _ = Text::new("Fix: Acquiring 3D...", Point::new(12, 90), text_style).draw(fb);
        let _ = Text::new("Toggle GPS switch to ON", Point::new(12, 106), text_style).draw(fb);
        let _ = Text::new("Standby: Active", Point::new(12, 120), text_style).draw(fb);
    }

    let _ = Text::new("Buttons: FK=TX TK=Page", Point::new(12, 145), text_style).draw(fb);
    let _ = Text::new("LEDs: Blue=RX/TX Red=Pwr", Point::new(12, 158), text_style).draw(fb);
}

/// Backward compatibility helper for render_dashboard
pub fn render_dashboard(node: &CompanionNode, fb: &mut EpdFrameBuffer) {
    render_page(node.current_page, node, fb);
}
