"""Primitive wrappers and MeshCore composition helpers for the oracle.

This module deliberately has no dependency on the Rust crate under test. The
cryptographic primitives come from `cryptography` (AES, standard Ed25519) and
`PyNaCl`/libsodium (Ed25519 point operations, X25519). Only the MeshCore
composition rules are written here.
"""

import hashlib
import hmac

import nacl.bindings as nb
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes


def sha256(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()


def sha512(data: bytes) -> bytes:
    return hashlib.sha512(data).digest()


def aes_ecb_encrypt(key: bytes, data: bytes) -> bytes:
    enc = Cipher(algorithms.AES(key), modes.ECB()).encryptor()
    return enc.update(data) + enc.finalize()


def aes_ecb_decrypt(key: bytes, data: bytes) -> bytes:
    dec = Cipher(algorithms.AES(key), modes.ECB()).decryptor()
    return dec.update(data) + dec.finalize()


def ed25519_pub(seed: bytes) -> bytes:
    sk = Ed25519PrivateKey.from_private_bytes(seed)
    return sk.public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw
    )


def ed25519_sign(seed: bytes, message: bytes) -> bytes:
    return Ed25519PrivateKey.from_private_bytes(seed).sign(message)


def expand(seed: bytes) -> bytes:
    """MeshCore/orlp 64-byte expanded key: clamped SHA-512(seed)."""
    if len(seed) != 32:
        raise ValueError("seed must be 32 bytes")
    h = bytearray(sha512(seed))
    h[0] &= 248
    h[31] &= 63
    h[31] |= 64
    return bytes(h)


def pub_from_expanded(prv: bytes) -> bytes:
    """Ed25519 public key from the clamped scalar (64-byte expanded key)."""
    if len(prv) != 64:
        raise ValueError("expanded key must be 64 bytes")
    return nb.crypto_scalarmult_ed25519_base_noclamp(prv[:32])


def shared_secret(seed: bytes, peer_pub: bytes) -> bytes:
    """MeshCore ECDH: X25519(clamped scalar, ed25519_pub -> montgomery u)."""
    scalar = expand(seed)[:32]
    u = nb.crypto_sign_ed25519_pk_to_curve25519(peer_pub)
    return nb.crypto_scalarmult(scalar, u)


def seal(secret: bytes, plain: bytes) -> bytes:
    """MAC(2) || AES-128-ECB(zero-padded plaintext)."""
    if len(secret) != 32:
        raise ValueError("secret must be 32 bytes")
    pad = (-len(plain)) % 16
    ct = aes_ecb_encrypt(secret[:16], plain + b"\x00" * pad)
    mac = hmac.new(secret, ct, hashlib.sha256).digest()[:2]
    return mac + ct


def open_seal(secret: bytes, sealed: bytes) -> bytes:
    if len(secret) != 32:
        raise ValueError("secret must be 32 bytes")
    mac, ct = sealed[:2], sealed[2:]
    if not ct or len(ct) % 16:
        raise ValueError("ciphertext must be a non-empty multiple of 16")
    want = hmac.new(secret, ct, hashlib.sha256).digest()[:2]
    if not hmac.compare_digest(mac, want):
        raise ValueError("bad MAC")
    return aes_ecb_decrypt(secret[:16], ct)


def txt_plain(ts: int, txt_type: int, attempt: int, text: bytes) -> bytes:
    """TXT_MSG plaintext: ts(4) || (txt_type<<2 | attempt&3) || text [|| 0, attempt]."""
    flags = ((txt_type & 0x3F) << 2) | (attempt & 3)
    out = ts.to_bytes(4, "little") + bytes([flags]) + text
    if attempt > 3:
        out += b"\x00" + bytes([attempt])
    return out


def ack_crc(ts: int, flags: int, text: bytes, sender_pub: bytes) -> int:
    """SHA256(ts || flags || text || sender_pub)[0..4] as a little-endian u32."""
    h = sha256(ts.to_bytes(4, "little") + bytes([flags]) + text + sender_pub)
    return int.from_bytes(h[:4], "little")


def packet(route: int, ptype: int, path_len: int, path: bytes, payload: bytes) -> bytes:
    """Wire packet for a non-transport route."""
    header = ((ptype & 0x0F) << 2) | (route & 0x03)
    return bytes([header, path_len]) + path + payload


# ---------------------------------------------------------------------------------------------
# MeshCore composition helpers

def le16(n: int) -> bytes:
    return (n & 0xFFFF).to_bytes(2, "little")


def le32(n: int) -> bytes:
    return (n & 0xFFFFFFFF).to_bytes(4, "little")


def utf8_prefix(data: bytes, limit: int) -> bytes:
    """Longest valid UTF-8 prefix of `data` that fits in `limit` bytes."""
    out = data[:limit]
    while out:
        try:
            out.decode("utf-8")
            return out
        except UnicodeDecodeError:
            out = out[:-1]
    return b""


def build_app_data(
    kind: int,
    name: bytes = b"",
    lat: int | None = None,
    lon: int | None = None,
    feat1: int | None = None,
    feat2: int | None = None,
) -> bytes:
    """Advert app_data: low nibble = kind, then optional fields, then name (<= 32 bytes)."""
    flags = kind & 0x0F
    out = bytearray([0])
    if lat is not None and lon is not None:
        flags |= 0x10
        out += int(lat).to_bytes(4, "little", signed=True)
        out += int(lon).to_bytes(4, "little", signed=True)
    if feat1:
        flags |= 0x20
        out += le16(feat1)
    if feat2:
        flags |= 0x40
        out += le16(feat2)
    if name:
        head = utf8_prefix(name, 32 - len(out))
        if head:
            flags |= 0x80
            out += head
    out[0] = flags
    return bytes(out)


def advert_payload(seed: bytes, pub: bytes, ts: int, app_data: bytes) -> bytes:
    """ADVERT: pub(32) || ts u32 || sig(64) || app_data, sig over pub||ts||app_data."""
    signed = pub + le32(ts) + app_data
    return pub + le32(ts) + ed25519_sign(seed, signed) + app_data


def channel_secret(psk16: bytes) -> bytes:
    if len(psk16) != 16:
        raise ValueError("PSK must be 16 bytes")
    return psk16 + b"\x00" * 16


def hashtag_secret(name: str) -> bytes:
    name = name.lstrip("#")
    return channel_secret(sha256(b"#" + name.encode("utf-8"))[:16])


def channel_hash(secret32: bytes) -> int:
    """SHA256(secret[0..16])[0] if the upper half is zero, else SHA256(secret)[0]."""
    if len(secret32) != 32:
        raise ValueError("channel secret must be 32 bytes")
    key = secret32[:16] if not any(secret32[16:]) else secret32
    return sha256(key)[0]


def region_key(name: str) -> bytes:
    name = name.lstrip("#")
    return sha256(b"#" + name.encode("utf-8"))[:16]


def transport_code(key16: bytes, ptype: int, payload: bytes) -> int:
    digest = hmac.new(key16, bytes([ptype & 0x0F]) + payload, hashlib.sha256).digest()
    code = int.from_bytes(digest[:2], "little")
    if code == 0:
        return 1
    if code == 0xFFFF:
        return 0xFFFE
    return code


def transport_packet(
    route: int,
    ptype: int,
    tc0: int,
    tc1: int,
    path_len: int,
    path: bytes,
    payload: bytes,
) -> bytes:
    header = ((ptype & 0x0F) << 2) | (route & 0x03)
    return (
        bytes([header])
        + le16(tc0)
        + le16(tc1)
        + bytes([path_len])
        + path
        + payload
    )


def packet_hash(ptype: int, path_len: int, payload: bytes) -> bytes:
    """Dedup hash: SHA256(type || [path_len u16 if TRACE] || payload)[0..8]."""
    pre = bytes([ptype & 0x0F])
    if ptype == 0x09:
        pre += le16(path_len)
    return sha256(pre + payload)[:8]


def ack_payload(crc: int, extended_attempt: int | None = None, random_byte: int = 0) -> bytes:
    """ACK: crc(4) || [extended attempt || random byte] (companion form)."""
    out = le32(crc)
    if extended_attempt is not None:
        out += bytes([extended_attempt & 0xFF, random_byte & 0xFF])
    return out


def group_txt_plain(ts: int, sender: bytes, text: bytes) -> bytes:
    """GRP_TXT plaintext: ts || 0x00 || "name: text" (160-byte text cap)."""
    text = text[: max(0, 160 - (len(sender) + 2))]
    return le32(ts) + b"\x00" + sender + b": " + text


def group_data_plain(data_type: int, data: bytes) -> bytes:
    """GRP_DATA plaintext: data_type u16 || length u8 || data (max 165 bytes)."""
    if len(data) > 165:
        raise ValueError("group data too long")
    return le16(data_type) + bytes([len(data)]) + data


def path_plain(
    path_len_byte: int,
    path: bytes,
    extra_type: int = 0,
    extra: bytes = b"",
    random4: bytes = b"\x00\x00\x00\x00",
) -> bytes:
    """PATH plaintext: path_len || path || extra_type || extra, or 0xFF || random(4)."""
    if extra:
        return bytes([path_len_byte]) + path + bytes([extra_type & 0x0F]) + extra
    return bytes([path_len_byte]) + path + b"\xff" + random4


def addressed_payload(dest: int, src: int, secret: bytes, plain: bytes) -> bytes:
    return bytes([dest & 0xFF, src & 0xFF]) + seal(secret, plain)


def anon_payload(dest: int, sender_pub: bytes, secret: bytes, plain: bytes) -> bytes:
    return bytes([dest & 0xFF]) + sender_pub + seal(secret, plain)


def group_payload(channel: int, secret: bytes, plain: bytes) -> bytes:
    return bytes([channel & 0xFF]) + seal(secret, plain)
