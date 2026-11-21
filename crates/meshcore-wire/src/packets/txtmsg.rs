//! Direct Text Message (`PAYLOAD_TYPE_TXT_MSG = 0x02`).
//!
//! Wire payload: `dest(1) ‖ src(1) ‖ seal(plain)`.
//! Plaintext: `ts u32 LE ‖ (txt_type << 2 | attempt & 3) ‖ text`,
//! with `0x00 ‖ attempt` appended when `attempt > 3`.

use heapless::String;

use crate::crypto::{
    ack_crc, ack_crc_signed, meshcore_encrypt_then_mac, meshcore_mac_then_decrypt,
};
use crate::packet::{Packet, PAYLOAD_TYPE_TXT_MSG};

pub const MAX_TEXT_LEN: usize = 160;
/// Largest AES-128-ECB padded ciphertext that fits in a TXT_MSG payload
/// (`dest(1) + src(1) + mac(2) + 176 = 180 <= MAX_PACKET_PAYLOAD(184)`).
const MAX_TXT_CIPHER: usize = 176;

pub const TXT_TYPE_PLAIN: u8 = 0;
pub const TXT_TYPE_CLI_DATA: u8 = 1;
pub const TXT_TYPE_SIGNED_PLAIN: u8 = 2;
pub const TXT_TYPE_CLI_COMMAND: u8 = 3;

#[derive(Clone, Debug, PartialEq)]
pub struct TxtMsgPacket {
    /// Destination node hash (`pub[0]`).
    pub dest: u8,
    /// Sender node hash (`pub[0]`).
    pub src: u8,
    pub timestamp: u32,
    pub txt_type: u8,
    pub attempt: u8,
    pub text: String<MAX_TEXT_LEN>,
    /// CRC the sender expects in the ACK.
    pub ack_crc: u32,
    /// Byte after the terminating NUL: the extended attempt, or padding.
    pub extended_attempt: u8,
    /// Signing-key prefix for `TXT_TYPE_SIGNED_PLAIN`; zero otherwise.
    pub signed_prefix: [u8; 4],
}

impl TxtMsgPacket {
    pub fn new(
        dest: u8,
        src: u8,
        timestamp: u32,
        txt_type: u8,
        attempt: u8,
        text: &str,
        sender_pub: &[u8; 32],
    ) -> Result<Self, ()> {
        if text.bytes().any(|b| b == 0) {
            return Err(());
        }
        if text.len() > MAX_TEXT_LEN || (attempt > 3 && text.len() > MAX_TEXT_LEN - 2) {
            return Err(());
        }

        let mut t = String::new();
        t.push_str(text)?;

        let flags = flags_of(txt_type, attempt);
        Ok(Self {
            dest,
            src,
            timestamp,
            txt_type: txt_type & 0x3F,
            attempt,
            text: t,
            ack_crc: ack_crc(timestamp, flags, text.as_bytes(), sender_pub),
            extended_attempt: if attempt > 3 { attempt } else { 0 },
            signed_prefix: [0; 4],
        })
    }

    /// Encode into `dest(1) ‖ src(1) ‖ seal(plain)`. Returns the wire length.
    pub fn encode(&self, secret: &[u8; 32], out: &mut [u8]) -> Result<usize, ()> {
        if out.len() < 2 {
            return Err(());
        }
        let mut plain = [0u8; MAX_TXT_CIPHER];
        let plain_len = self.write_plaintext(&mut plain)?;

        out[0] = self.dest;
        out[1] = self.src;
        let sealed = meshcore_encrypt_then_mac(secret, &mut out[2..], &plain[..plain_len])?;
        Ok(2 + sealed)
    }

    /// Decode and decrypt an addressed TXT_MSG payload.
    ///
    /// `sender_pub` is the candidate peer's public key (hashed for plain DM
    /// ACKs); `recipient_pub` is the receiver's own public key (hashed for
    /// `TXT_TYPE_SIGNED_PLAIN` room ACKs per upstream `BaseChatMesh.cpp`).
    pub fn decode(
        src: &[u8],
        secret: &[u8; 32],
        sender_pub: &[u8; 32],
        recipient_pub: &[u8; 32],
    ) -> Result<Self, ()> {
        // The C++ requires more than 4 bytes before it even attempts decrypt.
        if src.len() <= 4 {
            return Err(());
        }
        let dest = src[0];
        let src_hash = src[1];

        let mut plain = [0u8; MAX_TXT_CIPHER];
        let plain_len = meshcore_mac_then_decrypt(secret, &mut plain, &src[2..])?;
        // The C++ requires `len > 5`.
        if plain_len <= 5 {
            return Err(());
        }

        let timestamp = u32::from_le_bytes([plain[0], plain[1], plain[2], plain[3]]);
        let flags = plain[4];
        let txt_type = flags >> 2;

        let rest = &plain[5..plain_len];
        let signed = txt_type == TXT_TYPE_SIGNED_PLAIN && rest.len() >= 4;
        let mut signed_prefix = [0u8; 4];
        let text_rest = if signed {
            // Signed plaintext: [prefix 4][text...]
            signed_prefix.copy_from_slice(&rest[..4]);
            &rest[4..]
        } else {
            rest
        };
        let end = text_rest
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(text_rest.len());
        let text_str = core::str::from_utf8(&text_rest[..end]).map_err(|_| ())?;
        let mut text = String::new();
        text.push_str(text_str)?;

        let extended_attempt = if signed {
            0
        } else {
            text_rest.get(end + 1).copied().unwrap_or(0)
        };
        let attempt = if extended_attempt > 3 {
            extended_attempt
        } else {
            flags & 0x03
        };

        let crc = if signed {
            ack_crc_signed(
                timestamp,
                flags,
                &signed_prefix,
                text.as_bytes(),
                recipient_pub,
            )
        } else {
            ack_crc(timestamp, flags, text.as_bytes(), sender_pub)
        };

        Ok(Self {
            dest,
            src: src_hash,
            timestamp,
            txt_type,
            attempt,
            ack_crc: crc,
            text,
            extended_attempt,
            signed_prefix,
        })
    }

    /// Pack into a complete MeshCore Packet.
    pub fn to_packet(&self, route_type: u8, secret: &[u8; 32]) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_TXT_MSG);
        let mut buf = [0u8; 184];
        let len = self.encode(secret, &mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }

    fn write_plaintext(&self, out: &mut [u8]) -> Result<usize, ()> {
        let text = self.text.as_bytes();
        let flags = flags_of(self.txt_type, self.attempt);
        let mut n = 0;

        let signed = self.txt_type == TXT_TYPE_SIGNED_PLAIN;
        if out.len()
            < 5 + if signed { 4 } else { 0 } + text.len() + if self.attempt > 3 { 2 } else { 0 }
        {
            return Err(());
        }
        out[0..4].copy_from_slice(&self.timestamp.to_le_bytes());
        out[4] = flags;
        n += 5;
        if signed {
            out[n..n + 4].copy_from_slice(&self.signed_prefix);
            n += 4;
        }
        out[n..n + text.len()].copy_from_slice(text);
        n += text.len();
        if self.attempt > 3 {
            out[n] = 0;
            out[n + 1] = self.attempt;
            n += 2;
        }
        Ok(n)
    }
}

#[inline]
fn flags_of(txt_type: u8, attempt: u8) -> u8 {
    ((txt_type & 0x3F) << 2) | (attempt & 0x03)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_room_ack_hashes_recipient_key_while_plain_hashes_sender_key() {
        let secret = [0x33u8; 32];
        let room_pub = [0xAAu8; 32];
        let client_pub = [0xBBu8; 32];
        let author_prefix = [0x11, 0x22, 0x33, 0x44];
        let ts = 1_700_000_123;

        // Signed room post sent from room_pub to client_pub.
        let mut signed = TxtMsgPacket::new(
            client_pub[0],
            room_pub[0],
            ts,
            TXT_TYPE_SIGNED_PLAIN,
            0,
            "hello room",
            &room_pub,
        )
        .unwrap();
        signed.signed_prefix = author_prefix;

        let mut wire = [0u8; 184];
        let len = signed.encode(&secret, &mut wire).unwrap();
        let decoded = TxtMsgPacket::decode(&wire[..len], &secret, &room_pub, &client_pub).unwrap();

        let flags = flags_of(TXT_TYPE_SIGNED_PLAIN, 0);
        let expected_signed_crc =
            ack_crc_signed(ts, flags, &author_prefix, b"hello room", &client_pub);
        let wrong_sender_crc =
            ack_crc_signed(ts, flags, &author_prefix, b"hello room", &room_pub);
        assert_eq!(decoded.ack_crc, expected_signed_crc);
        assert_ne!(decoded.ack_crc, wrong_sender_crc);
        assert_eq!(decoded.signed_prefix, author_prefix);
        assert_eq!(decoded.text.as_str(), "hello room");

        // Plain DM still hashes sender_pub.
        let plain = TxtMsgPacket::new(
            client_pub[0],
            room_pub[0],
            ts,
            TXT_TYPE_PLAIN,
            0,
            "hello dm",
            &room_pub,
        )
        .unwrap();
        let len = plain.encode(&secret, &mut wire).unwrap();
        let decoded = TxtMsgPacket::decode(&wire[..len], &secret, &room_pub, &client_pub).unwrap();
        assert_eq!(
            decoded.ack_crc,
            ack_crc(ts, flags_of(TXT_TYPE_PLAIN, 0), b"hello dm", &room_pub)
        );
    }

    #[test]
    fn max_length_txt_msg_roundtrips_with_176_byte_ciphertext() {
        let secret = [0x55u8; 32];
        let sender_pub = [0x10u8; 32];
        let recipient_pub = [0x20u8; 32];
        let max_text = "a".repeat(MAX_TEXT_LEN);

        // Plain 160-byte text: 5 + 160 = 165B plaintext -> 176B ciphertext -> 180B wire payload.
        let msg = TxtMsgPacket::new(
            recipient_pub[0],
            sender_pub[0],
            42,
            TXT_TYPE_PLAIN,
            1,
            &max_text,
            &sender_pub,
        )
        .unwrap();
        let mut wire = [0u8; 184];
        let len = msg.encode(&secret, &mut wire).unwrap();
        assert_eq!(len, 2 + 2 + 176);
        let decoded =
            TxtMsgPacket::decode(&wire[..len], &secret, &sender_pub, &recipient_pub).unwrap();
        assert_eq!(decoded.text.as_str(), max_text);
        assert_eq!(decoded.attempt, 1);

        // Extended attempt (>3) at max allowed length (158B): 5 + 158 + 2 = 165B -> 176B ciphertext.
        let ext_text = "b".repeat(MAX_TEXT_LEN - 2);
        let ext_msg = TxtMsgPacket::new(
            recipient_pub[0],
            sender_pub[0],
            43,
            TXT_TYPE_PLAIN,
            5,
            &ext_text,
            &sender_pub,
        )
        .unwrap();
        let len = ext_msg.encode(&secret, &mut wire).unwrap();
        assert_eq!(len, 2 + 2 + 176);
        let decoded =
            TxtMsgPacket::decode(&wire[..len], &secret, &sender_pub, &recipient_pub).unwrap();
        assert_eq!(decoded.text.as_str(), ext_text);
        assert_eq!(decoded.attempt, 5);

        // Signed 160-byte text: 5 + 4 + 160 = 169B plaintext -> 176B ciphertext.
        let mut signed = TxtMsgPacket::new(
            recipient_pub[0],
            sender_pub[0],
            44,
            TXT_TYPE_SIGNED_PLAIN,
            0,
            &max_text,
            &sender_pub,
        )
        .unwrap();
        signed.signed_prefix = [1, 2, 3, 4];
        let len = signed.encode(&secret, &mut wire).unwrap();
        assert_eq!(len, 2 + 2 + 176);
        let decoded =
            TxtMsgPacket::decode(&wire[..len], &secret, &sender_pub, &recipient_pub).unwrap();
        assert_eq!(decoded.text.as_str(), max_text);
        assert_eq!(decoded.signed_prefix, [1, 2, 3, 4]);
    }
}
