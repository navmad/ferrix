//! Text CLI shared by the USB sideband and `CMD_RUN_CLI_COMMAND`.
//!
//! The command table is platform independent. Output goes through
//! [`TextSink`]; persistence, radio stats and platform actions go through
//! [`CliHost`].

use core::fmt::Write;

use defmt::info;

use heapless::String;

use crate::command::{UsbCommand, USB_COMMAND_CHANNEL};
use crate::companion::protocol::{CompanionHost, CompanionNode};
use crate::state::*;
use crate::types::*;

/// Output sink for one CLI line.
#[allow(async_fn_in_trait)]
pub trait TextSink {
    async fn send(&mut self, data: &[u8]) -> Result<(), ()>;
}

/// Collect CLI output into a bounded string for `PACKET_CLI_REPLY`.
pub struct TextBuffer<'a>(pub &'a mut String<256>);

impl TextSink for TextBuffer<'_> {
    async fn send(&mut self, data: &[u8]) -> Result<(), ()> {
        let text = core::str::from_utf8(data).map_err(|_| ())?;
        let avail = self.0.capacity() - self.0.len();
        if text.len() <= avail {
            let _ = self.0.push_str(text);
        } else {
            let mut end = avail;
            while end > 0 && !text.is_char_boundary(end) {
                end -= 1;
            }
            let _ = self.0.push_str(&text[..end]);
        }
        Ok(())
    }
}

/// Flash-wear counters shown by `stats-flash`.
pub struct StorageStats {
    pub contact_saves: u32,
    pub contact_erases: u32,
    pub message_records: u32,
    pub message_page_erases: u32,
    pub message_queued: u32,
    pub advert_records: u32,
    pub advert_erases: u32,
    pub config_saves: u32,
    pub config_erases: u32,
    /// Board uses external QSPI for contacts/messages.
    pub external_profile: bool,
    /// External flash is missing or unusable this session.
    pub external_unavailable: bool,
    /// External-store operations skipped because of the outage.
    pub external_drops: u32,
}

/// Transmit scheduler counters shown by `stats-tx`.
pub struct RadioStats {
    pub override_ms: u32,
    pub slot_ms: u32,
    pub used: u32,
    pub forward_scheduled: u32,
    pub forward_sent: u32,
    pub duplicate_deltas: [u32; 6],
}

/// Live power regulator and low-frequency clock telemetry.
pub struct PowerClockStats {
    pub lfclk_config: &'static str,
    pub lfclk_running: bool,
    pub lfclk_src: &'static str,
    pub lfclkstat_raw: u32,
    pub dcdc_reg1_configured: bool,
    pub dcdc_reg1_active: bool,
    pub dcdc_reg0_active: bool,
    pub sd_dcdc_result: u32,
}

/// Board services used by the text CLI.
pub trait CliHost: CompanionHost {
    fn leds_disabled(&self) -> bool;
    fn set_leds_disabled(&self, disabled: bool);
    fn storage_stats(&self) -> StorageStats;
    fn radio_stats(&self) -> RadioStats;
    fn power_clock_stats(&self) -> Option<PowerClockStats> {
        None
    }
    fn set_tx_slot_override(&self, ms: u32);
    fn clear_radio_stats(&self);
    fn advert_stats(&self) -> (usize, usize, bool);
    fn advert_slots(&self) -> usize;
    fn gps_enabled(&self) -> bool;
    fn set_gps_enabled(&self, on: bool);
    fn reboot_to_bootloader(&self) -> !;
    fn reboot_to_ota_dfu(&self) -> !;
    /// One-line board description for the `board` command.
    fn board_line(&self) -> &'static str;
}

/// Split an optional 2-character companion CLI tag prefix (`"XX|<cmd>"`) from
/// the command body.
pub(crate) fn split_cli_tag_prefix(line: &str) -> (&str, &str) {
    let trimmed = line.trim_start_matches(' ');
    if trimmed.len() > 4 && trimmed.as_bytes().get(2) == Some(&b'|') {
        (&trimmed[..3], &trimmed[3..])
    } else {
        ("", trimmed)
    }
}

/// Normalize a `PACKET_CLI_REPLY` buffer in place so `reply` does not hold a
/// second buffer across `.await` inside the Embassy task arena.
///
/// `body_start` is the byte offset immediately after any echoed `"XX|"` tag
/// prefix. Strips trailing `\r`/`\n` bytes and a leading `"  -> "` USB prompt.
pub(crate) fn normalize_cli_reply_in_place<const N: usize>(
    out: &mut String<N>,
    body_start: usize,
) {
    while out.len() > body_start && matches!(out.as_bytes().last(), Some(b'\r' | b'\n')) {
        out.pop();
    }
    const PROMPT: &[u8] = b"  -> ";
    if out
        .as_bytes()
        .get(body_start..)
        .is_some_and(|rest| rest.starts_with(PROMPT))
    {
        let new_len = out.len() - PROMPT.len();
        // SAFETY: `body_start` and `body_start + PROMPT.len()` are ASCII
        // boundaries within valid UTF-8, so shifting left preserves UTF-8.
        unsafe {
            let v = out.as_mut_vec();
            v.copy_within(body_start + PROMPT.len().., body_start);
            v.truncate(new_len);
        }
    }
}

/// Write a string slice escaped for embedding inside a JSON string literal.
pub(crate) fn write_json_escaped<W: Write>(out: &mut W, s: &str) {
    for ch in s.chars() {
        match ch {
            '"' => {
                let _ = out.write_str("\\\"");
            }
            '\\' => {
                let _ = out.write_str("\\\\");
            }
            c if (c as u32) < 0x20 => {}
            c => {
                let _ = out.write_char(c);
            }
        }
    }
}

/// Parse a finite signed coordinate in `[-max_abs, max_abs]`.
pub(crate) fn parse_signed_coord(val: &str, max_abs: f32) -> Option<f32> {
    let v = val.trim().parse::<f32>().ok()?;
    if v.is_finite() && v >= -max_abs && v <= max_abs {
        Some(v)
    } else {
        None
    }
}

/// Run one local CLI line and collect its reply for `PACKET_CLI_REPLY`.
///
/// Supports the optional 2-character tag prefix (`"XX|<cmd>"`) used by
/// companion apps and strips the USB serial `"  -> "` prompt and trailing
/// `\r\n` so binary protocol parsers receive clean JSON / `> <value>` strings.
pub async fn reply(
    line: &str,
    node: &mut CompanionNode<'_>,
    host: &dyn CliHost,
    out: &mut String<256>,
) {
    let (tag_prefix, cmd) = split_cli_tag_prefix(line);
    let _ = out.push_str(tag_prefix);
    let body_start = out.len();

    let mut sink = TextBuffer(out);
    handle_text_cli(&mut sink, cmd, node, host, false).await;

    normalize_cli_reply_in_place(out, body_start);
}

/// - `910.525` (MHz, decimal)
/// - `910525` (kHz, integer)
/// - `910525000` (Hz)
/// - `910` (MHz)
pub(crate) fn parse_freq_hz(val: &str) -> Option<u32> {
    if val.contains('.') {
        let (int_part, frac_part) = val.split_once('.')?;
        let mhz: u32 = int_part.parse().ok()?;
        let mut frac = heapless::String::<6>::new();
        for c in frac_part.chars() {
            if !c.is_ascii_digit() || frac.len() == 6 {
                return None;
            }
            let _ = frac.push(c);
        }
        while frac.len() < 6 {
            let _ = frac.push('0');
        }
        let frac_num: u32 = frac.parse().ok()?;
        mhz.checked_mul(1_000_000)?.checked_add(frac_num)
    } else {
        let n: u32 = val.parse().ok()?;
        if n >= 100_000_000 {
            Some(n) // Hz
        } else if n >= 100_000 {
            Some(n.checked_mul(1000)?) // kHz
        } else if n >= 100 {
            Some(n.checked_mul(1_000_000)?) // MHz
        } else {
            None
        }
    }
}

/// Parse a bandwidth from the USB CLI. Accepts `62.5`, `62`, or `62500`.
fn parse_bw_khz(val: &str) -> Option<u32> {
    let val = val.trim_end_matches("kHz").trim_end_matches("k").trim();
    let exact = match val {
        "7.8" => Some(7),
        "10.4" => Some(10),
        "15.6" => Some(15),
        "20.8" => Some(20),
        "31.25" => Some(31),
        "41.67" => Some(41),
        "62.5" => Some(62),
        _ => None,
    };
    if exact.is_some() {
        return exact;
    }
    let n: u32 = val.parse().ok()?;
    // >= 7000 is Hz (62500 -> 62); smaller values are already kHz.
    let bw = if n >= 7_000 { n / 1000 } else { n };
    matches!(bw, 7 | 10 | 15 | 20 | 31 | 41 | 62 | 125 | 250 | 500).then_some(bw)
}

/// Parse a coding rate denominator: `5`, `4/5`, `6`, `4/6`, ...
fn parse_cr(val: &str) -> Option<u8> {
    match val.trim() {
        "5" | "4/5" => Some(5),
        "6" | "4/6" => Some(6),
        "7" | "4/7" => Some(7),
        "8" | "4/8" => Some(8),
        _ => None,
    }
}

/// Parse a decimal value into tenths (`9`, `9.0`, `2.5` -> 90, 90, 25).
fn parse_decimal_tenths(val: &str) -> Option<u16> {
    let val = val.trim().trim_end_matches('%').trim();
    let (int_s, frac_s) = match val.split_once('.') {
        Some((i, f)) => (i, Some(f)),
        None => (val, None),
    };
    let int: u16 = int_s.parse().ok()?;
    let frac: u16 = match frac_s {
        Some(f) if f.is_empty() => 0,
        Some(f) => {
            let b = *f.as_bytes().first()?;
            if !b.is_ascii_digit() {
                return None;
            }
            (b - b'0') as u16
        }
        None => 0,
    };
    Some(int.saturating_mul(10).saturating_add(frac))
}

/// Render a bandwidth with trailing zeros stripped: 62.5, 125, 250, 500, 31.25...
fn format_bw(bw_khz: u32) -> heapless::String<16> {
    let hz = crate::airtime::bw_khz_to_hz(bw_khz);
    let mut s = heapless::String::<16>::new();
    if hz % 1000 == 0 {
        let _ = write!(s, "{}", hz / 1000);
    } else if hz % 100 == 0 {
        let _ = write!(s, "{}.{}", hz / 1000, (hz % 1000) / 100);
    } else if hz % 10 == 0 {
        let _ = write!(s, "{}.{:02}", hz / 1000, (hz % 1000) / 10);
    } else {
        let _ = write!(s, "{}.{:03}", hz / 1000, hz % 1000);
    }
    s
}


#[inline]
fn hex_to_nibble(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'a'..=b'f' => b - b'a' + 10,
        b'A'..=b'F' => b - b'A' + 10,
        _ => 0,
    }
}

#[inline]
fn hex_to_nibble_opt(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}


pub async fn handle_text_cli<W: TextSink>(
    sender: &mut W,
    cmd_line: &str,
    node: &mut CompanionNode<'_>,
    host: &dyn CliHost,
    echo: bool,
) {
    let storage = host.storage_stats();
    let radio = host.radio_stats();
    let trimmed = cmd_line.trim();
    if trimmed.is_empty() {
        return;
    }

    // Echo the command line back to the host terminal (interactive CLI only).
    if echo {
        let _ = sender.send(cmd_line.as_bytes()).await;
        let _ = sender.send(b"\r\n").await;
    }

    match trimmed {
        "get name" | "name" => {
            let mut s: heapless::String<64> = heapless::String::new();
            let callsign = crate::state::NODE_CALLSIGN.lock(|c| c.borrow().clone());
            let _ = write!(s, "  -> > {}\r\n", callsign.as_str());
            let _ = sender.send( s.as_bytes()).await;
        }
        "get role" | "role" => {
            let _ = sender.send(b"  -> > companion\r\n").await;
        }
        "get public.key" | "get public_key" | "public.key" | "get pubkey" => {
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(s, "  -> > ");
            for b in &node.public_key {
                let _ = write!(s, "{:02x}", b);
            }
            let _ = write!(s, "\r\n");
            let _ = sender.send( s.as_bytes()).await;
        }
        "get prv.key" | "get private_key" | "prv.key" => {
            let mut s: heapless::String<256> = heapless::String::new();
            let _ = write!(s, "  -> > ");
            for b in &node.private_key {
                let _ = write!(s, "{:02x}", b);
            }
            let _ = write!(s, "\r\n");
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set prv.key ") => {
            let hex_str = s[12..].trim();
            let valid_hex = hex_str.bytes().all(|b| b.is_ascii_hexdigit());
            if valid_hex && hex_str.len() == 128 {
                // MeshCore's 64-byte expanded key (`scalar ‖ nonce_prefix`).
                let mut key = [0u8; 64];
                for i in 0..64 {
                    let hi = hex_to_nibble(hex_str.as_bytes()[i * 2]);
                    let lo = hex_to_nibble(hex_str.as_bytes()[i * 2 + 1]);
                    key[i] = (hi << 4) | lo;
                }
                match meshcore_wire::identity::LocalIdentity::from_private(
                    node.callsign.as_str(),
                    &key,
                ) {
                    Some(id) => {
                        node.private_key = key;
                        node.public_key = *id.pub_key();
                        set_identity(*id.pub_key(), key);
                        host.set_custom_priv_key(key);
                        info!("Node expanded private & public key updated via CLI");
                        let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPrivateKey(key));
                        let _ = sender.send( b"  -> OK\r\n").await;
                    }
                    None => {
                        let _ = sender.send( b"  -> Error, reserved public key\r\n").await;
                    }
                }
            } else if valid_hex && hex_str.len() == 64 {
                // Convenience: accept a 32-byte seed and expand it for storage.
                let mut seed = [0u8; 32];
                for i in 0..32 {
                    let hi = hex_to_nibble(hex_str.as_bytes()[i * 2]);
                    let lo = hex_to_nibble(hex_str.as_bytes()[i * 2 + 1]);
                    seed[i] = (hi << 4) | lo;
                }
                let id = meshcore_wire::identity::LocalIdentity::from_seed(
                    node.callsign.as_str(),
                    &seed,
                );
                let key = id.private_key_bytes();
                node.private_key = key;
                node.public_key = *id.pub_key();
                set_identity(*id.pub_key(), key);
                host.set_custom_priv_key(key);
                info!("Node seed imported and expanded; public key updated via CLI");
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPrivateKey(key));
                let _ = sender.send( b"  -> OK\r\n").await;
            } else {
                let _ = sender.send( b"  -> Error, expected 64 or 128 hex chars\r\n").await;
            }
        }
        "set name" => {
            let _ = sender.send( b"  -> Error, bad chars\r\n").await;
        }
        s if s.starts_with("set name ") || s.starts_with("name ") => {
            let new_name = if let Some(rest) = s.strip_prefix("set name ") {
                rest.trim()
            } else if let Some(rest) = s.strip_prefix("name ") {
                rest.trim()
            } else {
                ""
            };

            let is_valid = !new_name.is_empty()
                && new_name.len() <= 31
                && !new_name.bytes().any(|b| {
                    b < 0x20
                        || b > 0x7E
                        || b == b'['
                        || b == b']'
                        || b == b'\\'
                        || b == b':'
                        || b == b','
                        || b == b'?'
                        || b == b'*'
                });

            if is_valid {
                host.set_callsign(new_name);
                node.callsign.clear();
                let _ = node.callsign.push_str(new_name);
                info!("Node callsign updated to: {}", new_name);
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetName(node.callsign.clone()));
                let _ = sender.send( b"  -> OK\r\n").await;
            } else {
                let _ = sender.send( b"  -> Error, bad chars\r\n").await;
            }
        }
        "ver" => {
            let mut s: heapless::String<48> = heapless::String::new();
            let _ = write!(s, "  -> {}\r\n", crate::version::FIRMWARE_VERSION);
            let _ = sender.send(s.as_bytes()).await;
        }
        "board" => {
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "  -> {}\r\n", host.board_line());
            let _ = sender.send( s.as_bytes()).await;
        }
        "get freq" => {
            let f = RADIO_SETTINGS.lock(|r| r.borrow().frequency_hz);
            let mut s: heapless::String<48> = heapless::String::new();
            let _ = write!(s, "> {:.3}\r\n", f as f32 / 1_000_000.0);
            let _ = sender.send( s.as_bytes()).await;
        }
        "get radio" | "radio" => {
            let (f, sf, bw, cr) = RADIO_SETTINGS.lock(|r| {
                let cfg = r.borrow();
                (cfg.frequency_hz, cfg.sf, cfg.bw_khz, cfg.cr)
            });
            let mut s: heapless::String<64> = heapless::String::new();
            let bw_str = format_bw(bw);
            let _ = write!(s, "> {:.3},{},{},{}\r\n", f as f32 / 1_000_000.0, bw_str.as_str(), sf, cr);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s == "set radio" || s.starts_with("set radio ") => {
            let arg = s["set radio".len()..].trim();
            let parsed = if arg.eq_ignore_ascii_case("default") {
                Some((910_525_000u32, 62u32, 7u8, 5u8))
            } else if arg.is_empty() {
                None
            } else {
                let mut parts = arg
                    .split(|c: char| c == ',' || c.is_whitespace())
                    .filter(|p| !p.is_empty());
                match (parts.next(), parts.next(), parts.next(), parts.next(), parts.next()) {
                    (Some(f), Some(b), Some(sf), Some(cr), None) => {
                        match (parse_freq_hz(f), parse_bw_khz(b), sf.parse::<u8>().ok(), parse_cr(cr)) {
                            (Some(freq), Some(bw), Some(sf), Some(cr))
                                if (5..=12).contains(&sf) =>
                            {
                                Some((freq, bw, sf, cr))
                            }
                            _ => None,
                        }
                    }
                    _ => None,
                }
            };

            match parsed {
                Some((freq, bw, sf, cr)) if (150_000_000..=960_000_000).contains(&freq) => {
                    RADIO_SETTINGS.lock(|r| {
                        let mut st = r.borrow_mut();
                        st.frequency_hz = freq;
                        st.bw_khz = bw;
                        st.sf = sf;
                        st.cr = cr;
                        // The caller owns the new frequency, so the remembered
                        // off-grid home is no longer valid. Leaving an allowed
                        // band also clears the client-repeat flag.
                        st.offgrid_home_freq = 0;
                        if !is_offgrid_freq_hz(freq) {
                            st.client_repeat = false;
                        }
                    });
                    host.request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetRadio {
                        frequency_hz: freq,
                        bw_khz: bw,
                        sf,
                        cr,
                    });
                    let bw_str = format_bw(bw);
                    let mut resp: heapless::String<64> = heapless::String::new();
                    let _ = write!(
                        resp,
                        "OK - {:.3},{},{},{}\r\n",
                        freq as f32 / 1_000_000.0,
                        bw_str.as_str(),
                        sf,
                        cr
                    );
                    let _ = sender.send( resp.as_bytes()).await;
                }
                _ => {
                    let _ = sender.send(
                        b"Error: freq 150-960, bw 7-500, sf 5-12, cr 5-8, or default\r\n",
                    )
                    .await;
                }
            }
        }
        s if s.starts_with("set freq ") => {
            let val = s[9..].trim();
            // Accepts "910.525" (MHz), "910525" (kHz), or "910525000" (Hz).
            if let Some(f) = parse_freq_hz(val) {
                if f >= 150_000_000 && f <= 960_000_000 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().frequency_hz = f;
                    });
                    host.request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetFrequency(f));
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "OK - {:.3}\r\n", f as f32 / 1_000_000.0);
                    let _ = sender.send( s.as_bytes()).await;
                } else {
                    let _ = sender.send( b"Error: frequency out of range (150-960 MHz)\r\n").await;
                }
            } else {
                let _ = sender.send( b"Error: invalid frequency format\r\n").await;
            }
        }
        "get tx" => {
            let tx = RADIO_SETTINGS.lock(|r| r.borrow().tx_power_dbm);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", tx);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set tx ") => {
            if let Ok(tx) = s[7..].trim().parse::<i8>() {
                if tx >= -9 && tx <= 22 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().tx_power_dbm = tx;
                    });
                    host.request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetTxPower(tx));
                    let mut s: heapless::String<32> = heapless::String::new();
                    let _ = write!(s, "OK - tx power={} dBm\r\n", tx);
                    let _ = sender.send( s.as_bytes()).await;
                } else {
                    let _ = sender.send( b"Error: TX power must be between -9 and 22 dBm\r\n").await;
                }
            } else {
                let _ = sender.send( b"Error: invalid TX power number\r\n").await;
            }
        }
        "get sf" => {
            let sf = RADIO_SETTINGS.lock(|r| r.borrow().sf);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", sf);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set sf ") => {
            let val = s[7..].trim().trim_start_matches("sf").trim_start_matches("SF");
            if let Ok(sf) = val.parse::<u8>() {
                if sf >= 5 && sf <= 12 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().sf = sf;
                    });
                    host.request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetSpreadingFactor(sf));
                    let mut s: heapless::String<32> = heapless::String::new();
                    let _ = write!(s, "OK - {}\r\n", sf);
                    let _ = sender.send( s.as_bytes()).await;
                } else {
                    let _ = sender.send( b"Error: SF must be between 5 and 12\r\n").await;
                }
            } else {
                let _ = sender.send( b"Error: invalid SF\r\n").await;
            }
        }
        "get bw" => {
            let bw = RADIO_SETTINGS.lock(|r| r.borrow().bw_khz);
            let mut s: heapless::String<32> = heapless::String::new();
            let bw_str = format_bw(bw);
            let _ = write!(s, "> {}\r\n", bw_str.as_str());
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set bw ") => {
            let val = s[7..].trim();
            if let Some(bw) = parse_bw_khz(val) {
                RADIO_SETTINGS.lock(|r| {
                    r.borrow_mut().bw_khz = bw;
                });
                host.request_save_config();
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetBandwidth(bw));
                let mut s: heapless::String<32> = heapless::String::new();
                let bw_str = format_bw(bw);
                let _ = write!(s, "OK - {}\r\n", bw_str.as_str());
                let _ = sender.send( s.as_bytes()).await;
            } else {
                let _ = sender.send( b"Error: supported BW are 7.8, 10.4, 15.6, 20.8, 31.25, 41.67, 62.5, 125, 250, 500 kHz (or Hz)\r\n").await;
            }
        }
        "get cr" => {
            let cr = RADIO_SETTINGS.lock(|r| r.borrow().cr);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", cr);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set cr ") => {
            let val = s[7..].trim();
            if let Some(cr) = parse_cr(val) {
                RADIO_SETTINGS.lock(|r| {
                    r.borrow_mut().cr = cr;
                });
                host.request_save_config();
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetCodingRate(cr));
                let mut s: heapless::String<32> = heapless::String::new();
                let _ = write!(s, "OK - 4/{}\r\n", cr);
                let _ = sender.send( s.as_bytes()).await;
            } else {
                let _ = sender.send( b"Error: supported CR values are 4/5, 4/6, 4/7, 4/8\r\n").await;
            }
        }
        "get path.hash.mode" | "get hash-mode" | "get hash-size" => {
            let mode = RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode);
            let mut resp = heapless::String::<64>::new();
            let _ = write!(resp, "> {}\r\n", mode);
            let _ = sender.send( resp.as_bytes()).await;
        }
        s if s.starts_with("set path.hash.mode ") || s.starts_with("set hash-mode ") => {
            let val_str = s.split_whitespace().last().unwrap_or("");
            if let Ok(mode) = val_str.parse::<u8>() {
                if mode <= 2 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().path_hash_mode = mode;
                    });
                    host.request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPathHashMode(mode));
                    let mut resp = heapless::String::<64>::new();
                    let _ = write!(resp, "OK\r\n");
                    let _ = sender.send( resp.as_bytes()).await;
                } else {
                    let _ = sender.send( b"Error: path.hash.mode must be 0, 1 or 2\r\n").await;
                }
            } else {
                let _ = sender.send( b"  -> Error: invalid number\r\n").await;
            }
        }
        s if s.starts_with("set hash-size ") => {
            let val_str = s.split_whitespace().last().unwrap_or("");
            if let Ok(size) = val_str.parse::<u8>() {
                if (1..=3).contains(&size) {
                    let mode = size - 1;
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().path_hash_mode = mode;
                    });
                    host.request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPathHashMode(mode));
                    let mut resp = heapless::String::<64>::new();
                    let _ = write!(resp, "OK\r\n");
                    let _ = sender.send( resp.as_bytes()).await;
                } else {
                    let _ = sender.send( b"Error: hash-size must be 1, 2 or 3 bytes\r\n").await;
                }
            } else {
                let _ = sender.send( b"  -> Error: invalid number\r\n").await;
            }
        }
        "get sound" | "get buzzer" => {
            let on = crate::state::buzzer_enabled();
            let mut s: heapless::String<16> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", if on { "on" } else { "off" });
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set sound ") || s.starts_with("set buzzer ") => {
            let val = s.rsplit_once(' ').map(|(_, v)| v.trim()).unwrap_or("");
            let on = matches!(val, "on" | "1" | "true");
            let off = matches!(val, "off" | "0" | "false");
            if !on && !off {
                let _ = sender.send(b"Error: must be on or off\r\n").await;
            } else {
                crate::state::set_buzzer_enabled(on);
                host.request_save_config();
                let msg: &[u8] = if on {
                    b"OK - sound is now ON\r\n"
                } else {
                    b"OK - sound is now OFF\r\n"
                };
                let _ = sender.send(msg).await;
            }
        }
        "get refresh.interval" | "get display.refresh" => {
            let secs = crate::state::display_refresh_interval_secs();
            let mut s: heapless::String<48> = heapless::String::new();
            if secs == 0 {
                let _ = write!(s, "  -> > 0 (on update)\r\n");
            } else {
                let _ = write!(s, "  -> > {} s\r\n", secs);
            }
            let _ = sender.send(s.as_bytes()).await;
        }
        s if s.starts_with("set refresh.interval ") || s.starts_with("set display.refresh ") => {
            let val = s.rsplit_once(' ').map(|(_, v)| v.trim()).unwrap_or("");
            match val.parse::<u16>() {
                Ok(secs) if crate::state::set_display_refresh_interval_secs(secs) => {
                    host.request_save_config();
                    let mut resp: heapless::String<64> = heapless::String::new();
                    if secs == 0 {
                        let _ = write!(resp, "  -> OK (refresh on update)\r\n");
                    } else {
                        let _ = write!(resp, "  -> OK (refresh every {} s)\r\n", secs);
                    }
                    let _ = sender.send(resp.as_bytes()).await;
                }
                _ => {
                    let _ = sender
                        .send(b"  -> Error: expected 0 (on update) or 300..=3600 seconds\r\n")
                        .await;
                }
            }
        }
        "get multiacks" | "get multi.acks" => {
            let val = crate::state::prefs().multi_acks.min(1);
            node.multi_acks = val;
            let mut s: heapless::String<16> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", val);
            let _ = sender.send(s.as_bytes()).await;
        }
        s if s.starts_with("set multiacks ") || s.starts_with("set multi.acks ") => {
            let val = s.rsplit_once(' ').map(|(_, v)| v.trim()).unwrap_or("");
            let parsed = if matches!(val, "on" | "1" | "true") {
                Some(1u8)
            } else if matches!(val, "off" | "0" | "false") {
                Some(0u8)
            } else {
                None
            };
            match parsed {
                Some(v) => {
                    node.multi_acks = v;
                    crate::state::update_prefs(|p| p.multi_acks = v);
                    host.request_save_config();
                    let msg: &[u8] = if v != 0 {
                        b"OK - multiacks is now ON\r\n"
                    } else {
                        b"OK - multiacks is now OFF\r\n"
                    };
                    let _ = sender.send(msg).await;
                }
                None => {
                    let _ = sender.send(b"Error: must be 0/1 or on/off\r\n").await;
                }
            }
        }
        "get repeat" => {
            // For a companion this reflects off-grid mode.
            let on = crate::state::offgrid_active();
            let mut s: heapless::String<16> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", if on { "on" } else { "off" });
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set repeat ") => {
            let val = s[11..].trim();
            let on = matches!(val, "on" | "1" | "true");
            let off = matches!(val, "off" | "0" | "false");
            if !on && !off {
                let _ = sender.send( b"Error: must be on or off\r\n").await;
            } else {
                let change = crate::state::offgrid_set(on);
                if change.no_band {
                    let _ = sender.send(
                        b"Error: no off-grid frequency in this band\r\n",
                    )
                    .await;
                } else {
                    if change.changed {
                        host.request_save_config();
                    }
                    if change.freq_changed {
                        let _ = host.request_radio_config();
                    }
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetRepeat(
                        crate::state::forwarding_enabled(),
                    ));
                    let msg: &[u8] = if on {
                        b"OK - repeat is now ON\r\n"
                    } else {
                        b"OK - repeat is now OFF\r\n"
                    };
                    let _ = sender.send( msg).await;
                }
            }
        }
        s if s.starts_with("send ") => {
            let msg = s[5..].trim();
            if msg.is_empty() {
                let _ = sender.send( b"  -> Error: message cannot be empty\r\n").await;
            } else {
                let mut s_msg = heapless::String::<160>::new();
                let _ = s_msg.push_str(msg);
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SendGroupText {
                    channel_idx: 0,
                    timestamp: get_device_time(),
                    text: s_msg,
                });
                RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                });
                let _ = sender.send( b"  -> OK (Group message broadcast queued)\r\n").await;
            }
        }
        s if s.starts_with("msg ") => {
            let rest = s[4..].trim();
            if let Some(space_idx) = rest.find(' ') {
                let hex_str = &rest[..space_idx];
                let msg = rest[space_idx + 1..].trim();
                if hex_str.len() == 8 {
                    let mut dest = [0u8; 4];
                    let mut valid = true;
                    for i in 0..4 {
                        if let (Some(hi), Some(lo)) = (
                            hex_to_nibble_opt(hex_str.as_bytes()[i * 2]),
                            hex_to_nibble_opt(hex_str.as_bytes()[i * 2 + 1]),
                        ) {
                            dest[i] = (hi << 4) | lo;
                        } else {
                            valid = false;
                            break;
                        }
                    }
                    if valid && !msg.is_empty() {
                        let prefix = DISCOVERED_CONTACTS.lock(|c| {
                            c.borrow()
                                .iter()
                                .find(|ct| &ct.pub_key[..4] == &dest[..])
                                .map(|ct| {
                                    let mut p = [0u8; 6];
                                    p.copy_from_slice(&ct.pub_key[..6]);
                                    p
                                })
                        });
                        match prefix {
                            Some(dest_prefix) => {
                                let mut s_msg = heapless::String::<160>::new();
                                if s_msg.push_str(msg).is_ok() {
                                    let _ = USB_COMMAND_CHANNEL.try_send(
                                        UsbCommand::SendDirectText {
                                            dest_prefix,
                                            timestamp: get_device_time(),
                                            txt_type: 0,
                                            attempt: 0,
                                            text: s_msg,
                                        },
                                    );
                                    RADIO_TELEMETRY.lock(|t| {
                                        let mut tel = t.borrow_mut();
                                        tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                                    });
                                    let mut out_str = heapless::String::<72>::new();
                                    let _ = write!(
                                        out_str,
                                        "  -> OK (Direct message queued to {})\r\n",
                                        hex_str
                                    );
                                    let _ = sender.send( out_str.as_bytes()).await;
                                } else {
                                    let _ = sender.send(
                                        b"  -> Error: message exceeds 160 bytes\r\n",
                                    )
                                    .await;
                                }
                            }
                            None => {
                                let _ = sender.send(
                                    b"  -> Error: no contact with that 4-byte hash\r\n",
                                )
                                .await;
                            }
                        }
                    } else {
                        let _ = sender.send( b"  -> Error: invalid hex ID or empty text\r\n").await;
                    }
                } else {
                    let _ = sender.send( b"  -> Error: target ID must be 8 hex characters (4 bytes)\r\n").await;
                }
            } else {
                let _ = sender.send( b"  -> Usage: msg <8-hex-id> <message>\r\n").await;
            }
        }
        "contacts" | "get contacts" => {
            let count = DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
            if count == 0 {
                let _ = sender.send( b"  -> No contacts discovered yet (listening for adverts...)\r\n").await;
            } else {
                let mut hdr = heapless::String::<64>::new();
                let _ = write!(hdr, "  -> Discovered Contacts ({}):\r\n", count);
                let _ = sender.send( hdr.as_bytes()).await;
                let _ = sender.send( b"  -> ID        TYPE  CALLSIGN        RSSI     SNR   LAT        LON        SEEN\r\n").await;
                for idx in 0..count {
                    let Some(c) = DISCOVERED_CONTACTS.lock(|c| c.borrow().get(idx).cloned()) else {
                        continue;
                    };
                    let ty = match c.contact_type {
                        2 => "RPT",
                        3 => "ROOM",
                        4 => "SENS",
                        _ => "CHAT",
                    };
                    let mut s = heapless::String::<128>::new();
                    let _ = write!(
                        s,
                        "  -> {:02x}{:02x}{:02x}{:02x}  {:<4}  {:<14}  {:>4}dBm  {:>3}dB  {:>10} {:>10}  {}s\r\n",
                        c.node_id[0], c.node_id[1], c.node_id[2], c.node_id[3],
                        ty,
                        c.callsign.as_str(),
                        c.rssi, c.snr, c.latitude, c.longitude, c.last_seen_epoch
                    );
                    let _ = sender.send( s.as_bytes()).await;
                }
            }
        }
        "get leds" => {
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {}\r\n", if host.leds_disabled() { "off" } else { "on" });
            let _ = sender.send( s.as_bytes()).await;
        }
        "set leds on" | "set leds off" => {
            let disabled = trimmed == "set leds off";
            host.set_leds_disabled(disabled);
            let _ = sender.send(
                if disabled {
                    b"  -> OK (LEDs off)\r\n" as &[u8]
                } else {
                    b"  -> OK (LEDs on)\r\n" as &[u8]
                },
            )
            .await;
        }
        "get rxduty" | "get rx.duty" => {
            let on = RADIO_SETTINGS.lock(|r| r.borrow().rx_duty_cycle);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {}\r\n", if on { "on" } else { "off" });
            let _ = sender.send( s.as_bytes()).await;
        }
        "set rxduty on" | "set rxduty off" => {
            let on = trimmed == "set rxduty on";
            RADIO_SETTINGS.lock(|r| r.borrow_mut().rx_duty_cycle = on);
            host.request_save_config();
            let _ = host.request_radio_config();
            let _ = sender.send(
                if on {
                    b"  -> OK (RX duty cycle on)\r\n" as &[u8]
                } else {
                    b"  -> OK (RX duty cycle off)\r\n" as &[u8]
                },
            )
            .await;
        }
        "get af" => {
            let af = RADIO_SETTINGS.lock(|r| r.borrow().airtime_factor_tenths);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {}.{:02}\r\n", af / 10, af % 10);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set af ") => {
            if let Some(af) = parse_decimal_tenths(&s[7..]).map(|v| v.min(90) as u8) {
                RADIO_SETTINGS.lock(|r| r.borrow_mut().airtime_factor_tenths = af);
                host.request_save_config();
                let mut out: heapless::String<48> = heapless::String::new();
                let _ = write!(out, "  -> OK - af={}.{:02}\r\n", af / 10, af % 10);
                let _ = sender.send( out.as_bytes()).await;
            } else {
                let _ = sender.send( b"Error: af must be between 0 and 9\r\n").await;
            }
        }
        "get dutycycle" | "get duty" => {
            let af = RADIO_SETTINGS.lock(|r| r.borrow().airtime_factor_tenths) as u32;
            let duty_x10 = 10_000 / (af + 10);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {}.{}%\r\n", duty_x10 / 10, duty_x10 % 10);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set dutycycle ") || s.starts_with("set duty ") => {
            let arg = s
                .strip_prefix("set dutycycle ")
                .or_else(|| s.strip_prefix("set duty "))
                .unwrap_or("")
                .trim();
            if let Some(dc_x10) = parse_decimal_tenths(arg) {
                let dc_x10 = dc_x10.clamp(100, 1000);
                let af = if dc_x10 <= 100 {
                    90u8
                } else {
                    ((10_000 / dc_x10 as u32) - 10).min(90) as u8
                };
                RADIO_SETTINGS.lock(|r| r.borrow_mut().airtime_factor_tenths = af);
                host.request_save_config();
                let duty_x10 = 10_000 / (af as u32 + 10);
                let mut out: heapless::String<64> = heapless::String::new();
                let _ = write!(
                    out,
                    "  -> OK - dutycycle={}.{}%\r\n",
                    duty_x10 / 10,
                    duty_x10 % 10
                );
                let _ = sender.send( out.as_bytes()).await;
            } else {
                let _ = sender.send( b"Error: dutycycle must be 10-100%\r\n").await;
            }
        }
        "get autoadd" | "get auto-add" => {
            let a = AUTOADD_SETTINGS.lock(|x| *x.borrow());
            let enabled = (a.manual_add_contacts & 1) == 0 || a.config != 0;
            let mode = if (a.manual_add_contacts & 1) == 0 { "all" } else { "types" };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "  -> auto-add: {} (mode={}, config=0x{:02X}, max_hops={})\r\n",
                if enabled { "on" } else { "off" },
                mode,
                a.config,
                a.max_hops
            );
            let _ = sender.send( s.as_bytes()).await;
        }
        "get companion" => {
            use core::sync::atomic::Ordering;
            let dq = crate::companion::protocol::DEVICE_QUERY_COUNT.load(Ordering::Relaxed);
            let ga = crate::companion::protocol::GET_AUTOADD_COUNT.load(Ordering::Relaxed);
            let sa = crate::companion::protocol::SET_AUTOADD_COUNT.load(Ordering::Relaxed);
            let lc = crate::companion::protocol::LAST_CMD.load(Ordering::Relaxed);
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(
                s,
                "  -> device_query={} get_autoadd={} set_autoadd={} last_cmd=0x{:02X}\r\n",
                dq, ga, sa, lc
            );
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s == "set autoadd" || s.starts_with("set autoadd ") => {
            let arg = s["set autoadd".len()..].trim();
            let (manual, config, max_hops, label) = if arg.eq_ignore_ascii_case("on") {
                (0u8, 0x00u8, 0u8, 1u8)
            } else if arg.eq_ignore_ascii_case("off") {
                (1u8, 0x00u8, 0u8, 2u8)
            } else {
                let mut parts = arg
                    .split(|c: char| c == ',' || c.is_whitespace())
                    .filter(|p| !p.is_empty());
                match (parts.next(), parts.next(), parts.next()) {
                    (Some(cfg), Some(hops), None) => {
                        let cfg_val = cfg
                            .trim_start_matches("0x")
                            .trim_start_matches("0X");
                        let config = u8::from_str_radix(cfg_val, 16)
                            .ok()
                            .filter(|v| *v <= 0x1F);
                        let hops = hops.parse::<u8>().ok().filter(|v| *v <= 64);
                        match (config, hops) {
                            (Some(config), Some(max_hops)) => (1, config, max_hops, 3),
                            _ => (0, 0, 0, 0),
                        }
                    }
                    _ => (0, 0, 0, 0),
                }
            };

            if label != 0 {
                AUTOADD_SETTINGS.lock(|a| {
                    *a.borrow_mut() = AutoAddSettings {
                        manual_add_contacts: manual,
                        config,
                        max_hops,
                    };
                });
                node.manual_add_contacts = manual;
                node.autoadd_config = config;
                node.autoadd_max_hops = max_hops;
                host.request_save_config();

                let mut s: heapless::String<96> = heapless::String::new();
                if label == 1 {
                    let _ = write!(s, "  -> OK (auto-add on)\r\n");
                } else if label == 2 {
                    let _ = write!(s, "  -> OK (auto-add off)\r\n");
                } else {
                    let _ = write!(
                        s,
                        "  -> OK (config=0x{:02X}, max_hops={})\r\n",
                        config, max_hops
                    );
                }
                let _ = sender.send( s.as_bytes()).await;
            } else {
                let _ = sender.send(
                    b"Error: use set autoadd on|off or set autoadd <config>,<max_hops>\r\n",
                )
                .await;
            }
        }
        "clear contacts" => {
            DISCOVERED_CONTACTS.lock(|c| {
                c.borrow_mut().clear();
            });
            crate::storage::contacts::queue_clear().await;
            let _ = sender.send( b"  -> OK (Contacts cleared)\r\n").await;
        }
        "messages" | "inbox" | "get messages" => {
            let total = MESSAGE_LOG.lock(|m| m.borrow().len());
            if total == 0 {
                let _ = sender.send( b"  -> No messages received yet\r\n").await;
            } else {
                let _ = sender.send( b"  -> === Recent Messages ===\r\n").await;
                // Stream in small chunks: a 256-entry clone would overflow the
                // CLI task stack.
                let mut start = 0usize;
                while start < total {
                    let mut chunk: heapless::Vec<InboxItem, 16> = heapless::Vec::new();
                    MESSAGE_LOG.lock(|m| {
                        let list = m.borrow();
                        if start >= list.len() {
                            return;
                        }
                        let end = (start + 16).min(list.len());
                        for msg in &list[start..end] {
                            let _ = chunk.push(msg.clone());
                        }
                    });
                    if chunk.is_empty() {
                        break;
                    }
                    for item in chunk.iter() {
                        let mut s = heapless::String::<320>::new();
                        match item {
                            InboxItem::Text(msg) => {
                                let mode = if msg.is_direct { "[DM]" } else { "[GRP]" };
                                let _ = write!(
                                    s,
                                    "  -> {} <{}> {}: {} (RSSI: {}dBm, SNR: {}dB)\r\n",
                                    mode,
                                    msg.sender.as_str(),
                                    msg.timestamp,
                                    msg.text.as_str(),
                                    msg.rssi,
                                    msg.snr
                                );
                            }
                            InboxItem::ChannelData(data) => {
                                let _ = write!(
                                    s,
                                    "  -> [DATA] ch {} type {} ({} bytes, SNR: {}dB)\r\n",
                                    data.channel_idx,
                                    data.data_type,
                                    data.data.len(),
                                    data.snr
                                );
                            }
                        }
                        let _ = sender.send( s.as_bytes()).await;
                    }
                    start += chunk.len();
                }
            }
        }
        "clear messages" => {
            MESSAGE_LOG.lock(|m| {
                m.borrow_mut().clear();
            });
            crate::storage::messages::queue_clear().await;
            let _ = sender.send( b"  -> OK (Messages cleared)\r\n").await;
        }
        "get tx.slot" => {
            let override_ms = radio.override_ms;
            let last = radio.slot_ms;
            let mut s: heapless::String<80> = heapless::String::new();
            if override_ms == 0 {
                let _ = write!(s, "  -> auto (last {} ms)\r\n", last);
            } else {
                let _ = write!(s, "  -> {} ms (last {})\r\n", override_ms, last);
            }
            let _ = sender.send(s.as_bytes()).await;
        }
        s if s.starts_with("set tx.slot ") => {
            match s[12..].trim().parse::<u32>() {
                Ok(ms) if ms <= 5000 => {
                    host.set_tx_slot_override(ms);
                    let _ = sender.send(b"  -> OK\r\n").await;
                }
                _ => {
                    let _ = sender
                        .send(b"  -> Error, expected: set tx.slot <ms> (0 = auto)\r\n")
                        .await;
                }
            }
        }
        "stats-tx" => {
            let override_ms = radio.override_ms;
            let slot_ms = radio.slot_ms;
            let used = radio.used;
            let scheduled = radio.forward_scheduled;
            let sent = radio.forward_sent;
            let (control, flood, background, stale, cancelled) =
                RADIO_TELEMETRY.lock(|t| {
                    let tel = t.borrow();
                    (
                        tel.tx_control,
                        tel.tx_flood,
                        tel.tx_background,
                        tel.tx_stale,
                        tel.tx_cancelled,
                    )
                });
            let d = radio.duplicate_deltas;
            let mut s: heapless::String<256> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"slot_ms\":{},\"auto\":{},\"used\":{},\"ctrl\":{},\"flood\":{},\"bg\":{},\"stale\":{},\"cancel\":{},\"fwd_sched\":{},\"fwd_sent\":{},\"dup\":[{},{},{},{},{},{}]}}\r\n",
                slot_ms,
                override_ms == 0,
                used,
                control,
                flood,
                background,
                stale,
                cancelled,
                scheduled,
                sent,
                d[0], d[1], d[2], d[3], d[4], d[5]
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-radio" => {
            let (tx_ms, rx_ms, rssi, snr, wait_ms, retries, dropped, sent) =
                RADIO_TELEMETRY.lock(|t| {
                    let tel = t.borrow();
                    (
                        tel.tx_air_ms,
                        tel.rx_air_ms,
                        tel.last_rssi,
                        tel.last_snr,
                        tel.tx_wait_max_ms,
                        tel.tx_retries,
                        tel.tx_dropped,
                        tel.tx_beacons,
                    )
                });
            let mut s: heapless::String<256> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"noise_floor\":-118,\"last_rssi\":{},\"last_snr\":{:.2},\"tx_air_secs\":{},\"rx_air_secs\":{},\"tx_sent\":{},\"tx_wait_ms\":{},\"retries\":{},\"dropped\":{}}}\r\n",
                rssi,
                snr as f32,
                tx_ms / 1000,
                rx_ms / 1000,
                sent,
                wait_ms,
                retries,
                dropped
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-flash" => {
            let external = if !storage.external_profile {
                "internal"
            } else if storage.external_unavailable {
                "unavailable"
            } else {
                "ready"
            };
            let mut s: heapless::String<256> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"contacts\":{{\"s\":{},\"e\":{}}},\"messages\":{{\"r\":{},\"e\":{},\"q\":{}}},\"adverts\":{{\"r\":{},\"e\":{}}},\"config\":{{\"s\":{},\"e\":{}}},\"ext\":\"{}\",\"ext_drops\":{}}}\r\n",
                storage.contact_saves,
                storage.contact_erases,
                storage.message_records,
                storage.message_page_erases,
                storage.message_queued,
                storage.advert_records,
                storage.advert_erases,
                storage.config_saves,
                storage.config_erases,
                external,
                storage.external_drops
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "qspi" | "stats-qspi" => {
            use core::sync::atomic::Ordering;
            let status = crate::state::QSPI_STATUS.load(Ordering::Relaxed);
            let id = crate::state::QSPI_ID.load(Ordering::Relaxed);
            let status_s = match status {
                0 => "not_initialized",
                1 => "ready",
                2 => "id_mismatch",
                _ => "io_error",
            };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"status\":\"{}\",\"id\":\"{:06x}\"}}\r\n",
                status_s, id
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-disc" => {
            use core::sync::atomic::Ordering;
            let mut s: heapless::String<192> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"tx\":{},\"tx_tag\":\"{:08x}\",\"pending\":{},\"rx\":{},\"rx_tag\":\"{:08x}\",\"out\":\"0x{:02x}\",\"in\":\"0x{:02x}\",\"rtt_ms\":{}}}\r\n",
                crate::companion::protocol::DISC_TX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_TX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_PENDING.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_RX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_RX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_RX_OUT.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_RX_IN.load(Ordering::Relaxed),
                crate::companion::protocol::DISC_RX_RTT_MS.load(Ordering::Relaxed)
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-binary" => {
            use core::sync::atomic::Ordering;
            let mut s: heapless::String<160> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"tx\":{},\"tx_tag\":\"{:08x}\",\"type\":\"0x{:02x}\",\"rx\":{},\"rx_tag\":\"{:08x}\",\"len\":{}}}\r\n",
                crate::companion::protocol::REQ_TX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::REQ_TX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::REQ_TX_TYPE.load(Ordering::Relaxed),
                crate::companion::protocol::REQ_RX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::REQ_RX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::REQ_RX_LEN.load(Ordering::Relaxed)
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-login" => {
            use core::sync::atomic::Ordering;
            let mut s: heapless::String<192> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"tx\":{},\"key\":\"{:08x}\",\"tx_tag\":\"{:08x}\",\"flood\":{},\"pending\":{},\"rx\":{},\"rx_tag\":\"{:08x}\",\"len\":{},\"result\":{}}}\r\n",
                crate::companion::protocol::LOGIN_TX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_TX_KEY.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_TX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_TX_FLOOD.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_PENDING.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_RX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_RX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_RX_LEN.load(Ordering::Relaxed),
                crate::companion::protocol::LOGIN_RESULT.load(Ordering::Relaxed)
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-scope" => {
            let p = crate::companion::protocol::prefs();
            let over = crate::companion::protocol::scope_override();
            let scope_name = core::str::from_utf8(
                &p.scope_name[..p.scope_name.iter().position(|&b| b == 0).unwrap_or(31)],
            )
            .unwrap_or("");
            let mut s: heapless::String<192> = heapless::String::new();
            let _ = write!(s, "  -> {{\"default\":\"");
            write_json_escaped(&mut s, scope_name);
            let _ = write!(
                s,
                "\",\"key\":\"{:02x}{:02x}\",\"scoped\":{},\"override\":{},\"force_unscoped\":{}}}\r\n",
                p.scope_key[0],
                p.scope_key[1],
                crate::companion::protocol::SCOPED_FLOODS
                    .load(core::sync::atomic::Ordering::Relaxed),
                over.0,
                over.1
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-trace" => {
            use core::sync::atomic::Ordering;
            let mut s: heapless::String<192> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"tx\":{},\"tx_tag\":\"{:08x}\",\"tx_flags\":{},\"path_len\":{},\"path0\":\"{:08x}\",\"rx\":{},\"rx_tag\":\"{:08x}\",\"rx_flags\":{},\"hashes\":{},\"snrs\":{}}}\r\n",
                crate::companion::protocol::TRACE_TX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_TX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_TX_FLAGS.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_TX_PATH_LEN.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_TX_PATH.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_RX_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_RX_TAG.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_RX_FLAGS.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_RX_HASHES.load(Ordering::Relaxed),
                crate::companion::protocol::TRACE_RX_SNRS.load(Ordering::Relaxed)
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-adverts" => {
            use core::sync::atomic::Ordering;
            let (saved, pending, erase_pending) = host.advert_stats();
            let mut s: heapless::String<192> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"saved\":{},\"pending\":{},\"erase_pending\":{},\"adverts\":{},\"last_key\":\"{:08x}\",\"last_ts\":{},\"blob\":{}}}\r\n",
                saved,
                pending,
                erase_pending,
                crate::companion::protocol::ADVERT_COUNT.load(Ordering::Relaxed),
                crate::companion::protocol::ADVERT_LAST_KEY.load(Ordering::Relaxed),
                crate::companion::protocol::ADVERT_LAST_TS.load(Ordering::Relaxed),
                crate::companion::protocol::ADVERT_LAST_HAD_BLOB.load(Ordering::Relaxed)
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "gps on" => {
            host.set_gps_enabled(true);
            let _ = sender.send( b"  -> GPS power on\r\n").await;
        }
        "gps off" => {
            host.set_gps_enabled(false);
            let _ = sender.send( b"  -> GPS power off\r\n").await;
        }
        "gps advert" => {
            let mode = match crate::state::prefs().advert_loc_policy {
                1 => "share",
                2 => "prefs",
                _ => "none",
            };
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {}\r\n", mode);
            let _ = sender.send(s.as_bytes()).await;
        }
        s if s.starts_with("gps advert ") => {
            let arg = s["gps advert ".len()..].trim();
            let policy = if arg.eq_ignore_ascii_case("none") {
                Some((0u8, "none"))
            } else if arg.eq_ignore_ascii_case("share") {
                Some((1u8, "share"))
            } else if arg.eq_ignore_ascii_case("prefs") {
                Some((2u8, "prefs"))
            } else {
                None
            };
            if let Some((val, label)) = policy {
                node.advert_loc_policy = val;
                crate::state::update_prefs(|p| p.advert_loc_policy = val);
                host.request_save_config();
                let mut s: heapless::String<48> = heapless::String::new();
                let _ = write!(s, "  -> OK - gps advert {}\r\n", label);
                let _ = sender.send(s.as_bytes()).await;
            } else {
                let _ = sender
                    .send(b"  -> Error: use gps advert none|share|prefs\r\n")
                    .await;
            }
        }
        "get lat" => {
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {:.6}\r\n", node.latitude);
            let _ = sender.send(s.as_bytes()).await;
        }
        s if s.starts_with("set lat ") => {
            if let Some(lat) = parse_signed_coord(&s[8..], 90.0) {
                node.latitude = lat;
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetAdvertLatLon {
                    lat: (lat * 1_000_000.0) as i32,
                    lon: (node.longitude * 1_000_000.0) as i32,
                });
                let _ = sender.send(b"  -> OK\r\n").await;
            } else {
                let _ = sender
                    .send(b"  -> Error: lat must be between -90 and 90\r\n")
                    .await;
            }
        }
        "get lon" => {
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> > {:.6}\r\n", node.longitude);
            let _ = sender.send(s.as_bytes()).await;
        }
        s if s.starts_with("set lon ") => {
            if let Some(lon) = parse_signed_coord(&s[8..], 180.0) {
                node.longitude = lon;
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetAdvertLatLon {
                    lat: (node.latitude * 1_000_000.0) as i32,
                    lon: (lon * 1_000_000.0) as i32,
                });
                let _ = sender.send(b"  -> OK\r\n").await;
            } else {
                let _ = sender
                    .send(b"  -> Error: lon must be between -180 and 180\r\n")
                    .await;
            }
        }
        "stats-wdt" | "watchdog" => {
            use core::sync::atomic::Ordering;
            let app = crate::state::APP_HEARTBEAT.load(Ordering::Relaxed);
            let storage = crate::state::STORAGE_HEARTBEAT.load(Ordering::Relaxed);
            let pets = crate::state::WDT_PETS.load(Ordering::Relaxed);
            let reason = crate::state::RESET_REASON.load(Ordering::Relaxed);
            let mut s: heapless::String<160> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"app\":{},\"storage\":{},\"pets\":{},\"reset\":\"0x{:08x}\",\"dog\":{}}}\r\n",
                app,
                storage,
                pets,
                reason,
                (reason & 0x2) != 0
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-gps" | "gps" => {
            let g = crate::state::gps_state();
            let now = embassy_time::Instant::now().as_millis() as u64;
            let age = now.saturating_sub(g.last_update_ms);
            let bytes = crate::state::GPS_BYTES.load(core::sync::atomic::Ordering::Relaxed);
            let sentences = crate::state::GPS_SENTENCES.load(core::sync::atomic::Ordering::Relaxed);
            let lat = if g.latitude.is_finite() { g.latitude } else { 0.0 };
            let lon = if g.longitude.is_finite() { g.longitude } else { 0.0 };
            let alt = if g.altitude_m.is_finite() { g.altitude_m } else { 0.0 };
            let hdop = if g.hdop.is_finite() { g.hdop } else { 0.0 };
            let mut s: heapless::String<224> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"power\":{},\"fix\":{},\"sats\":{},\"lat\":{:.6},\"lon\":{:.6},\"alt_m\":{:.1},\"hdop\":{:.1},\"age_ms\":{},\"bytes\":{},\"lines\":{}}}\r\n",
                host.gps_enabled(),
                g.fix,
                g.satellites,
                lat,
                lon,
                alt,
                hdop,
                age,
                bytes,
                sentences
            );
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-ble" | "get ble" => {
            use core::sync::atomic::Ordering;
            let st = crate::state::BLE_STATUS.load(Ordering::Relaxed);
            let mut s: heapless::String<224> = heapless::String::new();
            match st {
                0 => {
                    let _ = write!(s, "  -> {{\"state\":\"stopped\"}}\r\n");
                }
                1 => {
                    let _ = write!(s, "  -> {{\"state\":\"probing\"}}\r\n");
                }
                2 => {
                    let _ = write!(s, "  -> {{\"state\":\"starting\"}}\r\n");
                }
                3 => {
                    let connected = crate::state::BLE_CONNECTED.load(Ordering::Relaxed);
                    let callsign = crate::storage::get_callsign();
                    let _ = write!(s, "  -> {{\"state\":\"active\",\"name\":\"MeshCore-");
                    write_json_escaped(&mut s, callsign.as_str());
                    let _ = write!(
                        s,
                        "\",\"connected\":{},\"bonds\":{},\"loaded\":{},\"saved\":{},\"errors\":{},\"last_err\":{},\"recov\":{}}}\r\n",
                        connected,
                        crate::storage::bonds::count(),
                        crate::storage::bonds::BOND_LOADED.load(Ordering::Relaxed),
                        crate::storage::bonds::BOND_SAVES.load(Ordering::Relaxed),
                        crate::storage::bonds::BOND_SAVE_ERRORS.load(Ordering::Relaxed),
                        crate::storage::bonds::BOND_LAST_ERROR.load(Ordering::Relaxed),
                        crate::storage::bonds::BOND_RECOVERIES.load(Ordering::Relaxed)
                    );
                }
                code if code >= 0x1000 => {
                    let w0 = unsafe { *(0x1000 as *const u32) };
                    let w1 = unsafe { *(0x1004 as *const u32) };
                    let _ = write!(
                        s,
                        "  -> {{\"state\":\"sd_enable_err\",\"code\":\"0x{:04x}\",\"w0\":\"0x{:08x}\",\"w1\":\"0x{:08x}\"}}\r\n",
                        code & 0xFFF, w0, w1
                    );
                }
                code => {
                    let _ = write!(s, "  -> {{\"state\":\"error\",\"code\":{}}}\r\n", code);
                }
            }
            let _ = sender.send(s.as_bytes()).await;
        }
        "stats-power" => {
            if let Some(pw) = host.power_clock_stats() {
                let mut s: heapless::String<256> = heapless::String::new();
                let mut sd_ret = heapless::String::<16>::new();
                if pw.sd_dcdc_result == u32::MAX {
                    let _ = write!(sd_ret, "null");
                } else {
                    let _ = write!(sd_ret, "{}", pw.sd_dcdc_result);
                }
                let _ = write!(
                    s,
                    "  -> {{\"lfclk_cfg\":\"{}\",\"lfclk_running\":{},\"lfclk_src\":\"{}\",\"lfclkstat\":\"0x{:08x}\",\"dcdc_reg1_cfg\":{},\"dcdc_reg1\":{},\"dcdc_reg0\":{},\"sd_dcdc_ret\":{}}}\r\n",
                    pw.lfclk_config,
                    pw.lfclk_running,
                    pw.lfclk_src,
                    pw.lfclkstat_raw,
                    pw.dcdc_reg1_configured,
                    pw.dcdc_reg1_active,
                    pw.dcdc_reg0_active,
                    sd_ret.as_str()
                );
                let _ = sender.send(s.as_bytes()).await;
            } else {
                let _ = sender.send(b"  -> {\"error\":\"unsupported\"}\r\n").await;
            }
        }
        "stats-core" => {
            let uptime = embassy_time::Instant::now().as_secs();
            let mv = host.battery_millivolts();
            let pct = crate::battery::millivolts_to_percentage(host.battery_millivolts());
            let mut s: heapless::String<160> = heapless::String::new();
            let _ = write!(s, "  -> {{\"battery_mv\":{},\"battery_pct\":{},\"uptime_secs\":{},\"errors\":0,\"queue_len\":0,\"push_drops\":{}}}\r\n", mv, pct, uptime, crate::command::push_drops());
            let _ = sender.send( s.as_bytes()).await;
        }
        "battery" | "get battery" | "batt" | "get batt" => {
            let mv = host.battery_millivolts();
            let pct = crate::battery::millivolts_to_percentage(host.battery_millivolts());
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "  -> {} mV ({}%)\r\n", mv, pct);
            let _ = sender.send( s.as_bytes()).await;
        }
        "stats-packets" => {
            let (tx, rx) = RADIO_TELEMETRY.lock(|t| {
                let tel = t.borrow();
                (tel.tx_beacons, tel.rx_packets)
            });
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"recv\":{},\"sent\":{},\"flood_tx\":{},\"direct_tx\":0,\"flood_rx\":{},\"direct_rx\":0,\"recv_errors\":0}}\r\n",
                rx, tx, tx, rx
            );
            let _ = sender.send( s.as_bytes()).await;
        }
        "clear stats" => {
            RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.tx_beacons = 0;
                tel.rx_packets = 0;
                tel.tx_air_ms = 0;
                tel.rx_air_ms = 0;
                tel.has_rx = false;
                tel.tx_wait_max_ms = 0;
                tel.tx_control = 0;
                tel.tx_flood = 0;
                tel.tx_background = 0;
                tel.tx_stale = 0;
                tel.tx_cancelled = 0;
                tel.tx_retries = 0;
                tel.tx_dropped = 0;
            });
            host.clear_radio_stats();
            let _ = sender.send( b"  -> OK\r\n").await;
        }
        "advert" => {
            RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.tx_beacons = tel.tx_beacons.saturating_add(1);
            });
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::TriggerAdvert { flood: true });
            let _ = sender.send( b"  -> OK (MeshCore Advert broadcast queued)\r\n").await;
        }
        "advert.zerohop" => {
            RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.tx_beacons = tel.tx_beacons.saturating_add(1);
            });
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::TriggerAdvert { flood: false });
            let _ = sender.send(b"  -> OK - zerohop advert sent\r\n").await;
        }
        "time" | "get time" | "clock" => {
            let mut s: heapless::String<32> = heapless::String::new();
            let epoch = get_device_time();
            let _ = write!(s, "  -> {}\r\n", epoch);
            let _ = sender.send( s.as_bytes()).await;
        }
        s if s.starts_with("set time ") || s.starts_with("time ") => {
            let time_str = if s.starts_with("set time ") { &s[9..] } else { &s[5..] }.trim();
            let mut epoch = 0u64;
            for &b in time_str.as_bytes() {
                if b >= b'0' && b <= b'9' {
                    epoch = epoch.wrapping_mul(10).wrapping_add((b - b'0') as u64);
                }
            }
            node.epoch_time = epoch;
            set_device_time(epoch as u32);
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetTime(epoch));
            let _ = sender.send( b"  -> OK (Clock synchronized)\r\n").await;
        }
        "channels" | "channel" => {
            let mut buf: heapless::String<256> = heapless::String::new();
            let _ = write!(buf, "Channels:\r\n");
            CHANNELS.lock(|c| {
                let list = c.borrow();
                for (i, ch) in list.iter().enumerate() {
                    let name_len = ch.name.iter().position(|&b| b == 0).unwrap_or(32);
                    if name_len > 0 {
                        let name_str = core::str::from_utf8(&ch.name[..name_len]).unwrap_or("<invalid>");
                        let _ = write!(buf, "  [{}] {}\r\n", i, name_str);
                    }
                }
            });
            let _ = sender.send( buf.as_bytes()).await;
        }
        "clock sync" | "sync_time" => {
            let _ = sender.send( b"  -> OK\r\n").await;
        }
        "start ota" | "dfu ota" => {
            let _ = crate::storage::flush_pending().await;
            let _ = sender.send(b"  -> OK - rebooting to BLE OTA DFU\r\n").await;
            if USB_COMMAND_CHANNEL.try_send(UsbCommand::RebootToOtaDfu).is_err() {
                embassy_time::Timer::after_millis(100).await;
                host.reboot_to_ota_dfu();
            }
        }
        "start dfu" | "dfu" | "bootloader" => {
            let _ = crate::storage::flush_pending().await;
            let _ = sender.send( b"  -> Entering DFU bootloader...\r\n").await;
            embassy_time::Timer::after_millis(50).await;
            host.reboot_to_bootloader();
        }
        "reboot" => {
            let _ = crate::storage::flush_pending().await;
            let _ = sender.send( b"  -> Rebooting device...\r\n").await;
            embassy_time::Timer::after_millis(50).await;
            host.reboot();
        }
        "save" => {
            host.request_save_config();
            let _ = crate::storage::flush_pending().await;
            let _ = sender.send( b"  -> OK (Config saved to flash)\r\n").await;
        }
        "help" => {
            let _ = sender.send( b"  -> Commands:\r\n  ->   Identity: get/set name, get role, get/set lat/lon, get/set prv.key, get public.key\r\n  ->   Radio:    get radio, set radio <freq>,<bw>,<sf>,<cr>, get/set freq/tx/sf/bw/cr, get/set repeat, get/set rxduty, get/set af, get/set dutycycle, get/set hash-mode, stats-radio\r\n  ->   Mesh:     send <text>, msg <hex_id> <text>, messages, contacts, get/set autoadd, get/set multiacks, advert, advert.zerohop, gps advert, clock/time, channels\r\n  ->   System:   save, ver, board, get/set leds, get/set sound, get/set refresh.interval, stats-core, stats-ble, stats-gps, stats-power, stats-packets, clear stats/contacts/messages, start dfu, start ota, reboot\r\n").await;
        }
        _ => {
            let _ = sender.send( b"  -> Unknown command\r\n").await;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_reply_strips_usb_prompt_and_echoes_pipe_tag() {
        assert_eq!(split_cli_tag_prefix("stats-core"), ("", "stats-core"));
        assert_eq!(split_cli_tag_prefix("  01|stats-core"), ("01|", "stats-core"));
        assert_eq!(split_cli_tag_prefix("ab|get leds"), ("ab|", "get leds"));
        // Too short to have a command after `XX|`: treated as plain text.
        assert_eq!(split_cli_tag_prefix("ab|x"), ("", "ab|x"));

        let mut out = String::<256>::new();
        let _ = out.push_str("  -> {\"battery_mv\":4170,\"uptime_secs\":58}\r\n");
        normalize_cli_reply_in_place(&mut out, 0);
        assert_eq!(out.as_str(), "{\"battery_mv\":4170,\"uptime_secs\":58}");

        out.clear();
        let _ = out.push_str("01|");
        let start = out.len();
        let _ = out.push_str("  -> 1.18.1-ferrix\r\n");
        normalize_cli_reply_in_place(&mut out, start);
        assert_eq!(out.as_str(), "01|1.18.1-ferrix");

        out.clear();
        let _ = out.push_str("ab|");
        let start = out.len();
        let _ = out.push_str("  -> > on\r\n");
        normalize_cli_reply_in_place(&mut out, start);
        assert_eq!(out.as_str(), "ab|> on");

        out.clear();
        let _ = out.push_str("> 910.525,62.5,7,5\r\n");
        normalize_cli_reply_in_place(&mut out, 0);
        assert_eq!(out.as_str(), "> 910.525,62.5,7,5");
    }

    #[test]
    fn json_escaping_handles_quotes_backslashes_and_control_chars() {
        let mut out = String::<64>::new();
        write_json_escaped(&mut out, "a\"b\\c\n\rd");
        assert_eq!(out.as_str(), "a\\\"b\\\\cd");
    }

    #[test]
    fn signed_coord_parser_validates_bounds() {
        assert_eq!(parse_signed_coord("37.7749", 90.0), Some(37.7749));
        assert_eq!(parse_signed_coord(" -122.4194 ", 180.0), Some(-122.4194));
        assert_eq!(parse_signed_coord("90.0", 90.0), Some(90.0));
        assert_eq!(parse_signed_coord("-90.1", 90.0), None);
        assert_eq!(parse_signed_coord("180.1", 180.0), None);
        assert_eq!(parse_signed_coord("NaN", 90.0), None);
        assert_eq!(parse_signed_coord("abc", 90.0), None);
    }
}
