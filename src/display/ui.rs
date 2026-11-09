//! MeshCore UI Dashboard Renderer for 1.54" EPD
//!
//! Uses `embedded-graphics` primitives to display off-grid node telemetry.

use embedded_graphics::mono_font::ascii::{FONT_6X10, FONT_9X15_BOLD};
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle, Rectangle};
use embedded_graphics::text::Text;

use crate::companion::protocol::CompanionNode;
use crate::display::epd::EpdFrameBuffer;

pub fn render_dashboard(node: &CompanionNode, fb: &mut EpdFrameBuffer) {
    fb.clear();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let title_style = MonoTextStyle::new(&FONT_9X15_BOLD, BinaryColor::On);
    let line_style = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    // 1. Header Box
    let _ = Text::new(node.callsign, Point::new(4, 14), title_style).draw(fb);
    
    // Battery Percentage indicator
    let mut bat_str: heapless::String<12> = heapless::String::new();
    let _ = core::fmt::write(&mut bat_str, format_args!("{}%", node.battery_percentage));
    let _ = Text::new(&bat_str, Point::new(160, 14), text_style).draw(fb);

    // Header divider line
    let _ = Line::new(Point::new(0, 18), Point::new(199, 18))
        .into_styled(line_style)
        .draw(fb);

    // 2. Radio & Connectivity Status
    let _ = Text::new("Radio: SX1262 915MHz", Point::new(4, 30), text_style).draw(fb);
    let _ = Text::new("Link: USB / BLE NUS", Point::new(4, 42), text_style).draw(fb);

    // 3. GPS Position Status
    if node.gps_fix {
        let mut gps_str: heapless::String<32> = heapless::String::new();
        let _ = core::fmt::write(&mut gps_str, format_args!("GPS: {:.3}, {:.3}", node.latitude, node.longitude));
        let _ = Text::new(&gps_str, Point::new(4, 56), text_style).draw(fb);
    } else {
        let _ = Text::new("GPS: Acq Satellites...", Point::new(4, 56), text_style).draw(fb);
    }

    // Divider line
    let _ = Line::new(Point::new(0, 62), Point::new(199, 62))
        .into_styled(line_style)
        .draw(fb);

    // 4. Message Box Area
    let _ = Rectangle::new(Point::new(4, 68), Size::new(192, 106))
        .into_styled(line_style)
        .draw(fb);

    let _ = Text::new("MeshCore Companion", Point::new(12, 84), title_style).draw(fb);
    let _ = Text::new("Status: Ready", Point::new(12, 102), text_style).draw(fb);
    let _ = Text::new("Connect phone app or PC", Point::new(12, 116), text_style).draw(fb);
    let _ = Text::new("to send / receive msgs.", Point::new(12, 128), text_style).draw(fb);

    let mut unread_str: heapless::String<24> = heapless::String::new();
    let _ = core::fmt::write(&mut unread_str, format_args!("Unread: {} messages", node.unread_messages));
    let _ = Text::new(&unread_str, Point::new(12, 150), text_style).draw(fb);

    // 5. Footer Navigation Bar
    let _ = Line::new(Point::new(0, 180), Point::new(199, 180))
        .into_styled(line_style)
        .draw(fb);

    let _ = Text::new("FK: Ping | TK: Next Page", Point::new(12, 194), text_style).draw(fb);
}
