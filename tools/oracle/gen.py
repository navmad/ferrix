#!/usr/bin/env python3
"""Independent MeshCore vector oracle.

Composes the full over-the-air vector set from MeshCore's rules. The
cryptography comes from `cryptography` and `PyNaCl`; this script never imports
the Rust crate under test.

It emits the deterministic `oracle_vectors.toml` consumed by the Rust
conformance tests.

Usage:
    tools/oracle/.venv/bin/python tools/oracle/gen.py
    tools/oracle/.venv/bin/python tools/oracle/gen.py --emit --output PATH
"""

import argparse
import sys
from pathlib import Path

import selfcheck
from primitives import (
    ack_crc,
    ack_payload,
    addressed_payload,
    advert_payload,
    anon_payload,
    build_app_data,
    channel_hash,
    channel_secret,
    ed25519_pub,
    expand,
    group_data_plain,
    group_payload,
    group_txt_plain,
    hashtag_secret,
    le16,
    le32,
    packet,
    packet_hash,
    path_plain,
    region_key,
    seal,
    shared_secret,
    transport_code,
    transport_packet,
    txt_plain,
)

ROOT = Path(__file__).resolve().parents[2]

TS = 1_793_900_000
PUBLIC_PSK = bytes.fromhex("8b3387e9c5cdea6ac9e5edbaa115cd72")
CUSTOM_PSK = bytes.fromhex("9cd8fcf22a47333b591d96a2b848b73f")

# Fixed seeds, so every generated vector is deterministic.
SEEDS = {
    "alice": "b6ff817b2d5b3108fff31ef8c61c3207a08d098a12e2bc694c680abf276f6b6c",
    "bob": "8878fd3eeb1fd3a57d2006c66e3a3f33fbd5214ed03d2acb077cc014b46a60ea",
    "relay": "42b8405826bd0811fa33347459bd00fd3278867e5f4c6024cedc1f3b4c8e3a92",
}

REJECTS = [
    ("version_1", "490078"),
    ("path_hash_size_4", "09c10000000000"),
    (
        "path_too_long",
        "096800000000000000000000000000000000000000000000000000000000000000000000"
        "000000000000000000000000000000000000000000000000000000000000000000000000"
        "00000000000000000000000000000000000000000000000078",
    ),
    ("path_truncated", "09050102"),
    ("transport_truncated", "14010203"),
    ("payload_185", "1500" + "00" * 185),
    ("empty", ""),
]


def add_packet(
    vectors: dict,
    name: str,
    route: int,
    ptype: int,
    payload: bytes,
    path: bytes = b"",
    path_len: int = 0,
    transport: tuple[int, int] | None = None,
) -> bytes:
    if transport is None:
        raw = packet(route, ptype, path_len, path, payload)
    else:
        raw = transport_packet(
            route, ptype, transport[0], transport[1], path_len, path, payload
        )
    entry = {
        "name": name,
        "raw": raw.hex(),
        "route": route,
        "ptype": ptype,
        "path_len": path_len,
        "path": path.hex(),
        "payload": payload.hex(),
        "hash": packet_hash(ptype, path_len, payload).hex(),
    }
    if transport is not None:
        entry["transport"] = list(transport)
    vectors["packet"].append(entry)
    return raw


def build() -> dict:
    v: dict[str, list] = {
        key: []
        for key in [
            "identity",
            "ecdh",
            "seal",
            "channel",
            "packet",
            "advert",
            "txt",
            "ack",
            "group",
            "path",
            "anon",
            "req",
            "response",
            "trace",
            "multipart",
            "control",
            "raw",
            "transport",
            "reject",
        ]
    }

    ids = {}
    for name, seed_hex in SEEDS.items():
        seed = bytes.fromhex(seed_hex)
        prv = expand(seed)
        pub = ed25519_pub(seed)
        ids[name] = {"seed": seed, "prv": prv, "pub": pub}
        v["identity"].append(
            {"name": name, "seed": seed_hex, "prv": prv.hex(), "pub": pub.hex()}
        )

    secret_ab = shared_secret(ids["alice"]["seed"], ids["bob"]["pub"])
    secret_ar = shared_secret(ids["alice"]["seed"], ids["relay"]["pub"])
    v["ecdh"].append({"a": "alice", "b": "bob", "secret": secret_ab.hex()})

    for plain in (b"\x78", bytes(range(16)), bytes(range(40))):
        v["seal"].append(
            {
                "secret": secret_ab.hex(),
                "plain": plain.hex(),
                "sealed": seal(secret_ab, plain).hex(),
            }
        )

    public = channel_secret(PUBLIC_PSK)
    custom = channel_secret(CUSTOM_PSK)
    for name, secret in (
        ("Public", public),
        ("#test", hashtag_secret("test")),
        ("#fr", hashtag_secret("fr")),
    ):
        v["channel"].append(
            {"name": name, "secret": secret.hex(), "hash": channel_hash(secret)}
        )

    def add_advert(
        packet_name: str,
        identity: str,
        kind: int,
        label: bytes,
        lat: int | None = None,
        lon: int | None = None,
        feat1: int | None = None,
        feat2: int | None = None,
        path: bytes = b"",
        path_len: int = 0,
    ) -> None:
        app_data = build_app_data(kind, label, lat, lon, feat1, feat2)
        payload = advert_payload(
            ids[identity]["seed"], ids[identity]["pub"], TS, app_data
        )
        add_packet(v, packet_name, 1, 4, payload, path, path_len)
        entry = {
            "packet": packet_name,
            "identity": identity,
            "ts": TS,
            "app_data": app_data.hex(),
            "kind": kind,
            "adv_name": label.hex(),
        }
        if lat is not None and lon is not None:
            entry["lat"] = lat
            entry["lon"] = lon
        if feat1:
            entry["feat1"] = feat1
        if feat2:
            entry["feat2"] = feat2
        v["advert"].append(entry)

    add_advert("advert_alice", "alice", 1, b"Alice")
    add_advert(
        "advert_bob_loc",
        "bob",
        1,
        bytes.fromhex("426f6220636166c3a9"),
        lat=48_856_613,
        lon=2_352_222,
    )
    add_advert(
        "advert_relay_2hops",
        "relay",
        2,
        bytes.fromhex(
            "52656c616973204d6f6e746d617274726520e29bb0207472c3a873206c6f6e"
        ),
        path=bytes.fromhex("1234"),
        path_len=2,
    )
    add_advert(
        "advert_feat",
        "relay",
        2,
        b"Relay",
        lat=48_856_613,
        lon=2_352_222,
        feat1=0x1234,
        feat2=0x5678,
    )

    def add_txt(
        packet_name: str,
        frm: str,
        to: str,
        attempt: int,
        text: bytes,
        route: int = 1,
        path: bytes = b"",
        path_len: int = 0,
    ) -> int:
        a, b = ids[frm], ids[to]
        secret = shared_secret(a["seed"], b["pub"])
        plain = txt_plain(TS, 0, attempt, text)
        payload = addressed_payload(b["pub"][0], a["pub"][0], secret, plain)
        add_packet(v, packet_name, route, 2, payload, path, path_len)
        crc = ack_crc(TS, attempt & 3, text, a["pub"])
        v["txt"].append(
            {
                "packet": packet_name,
                "from": frm,
                "to": to,
                "ts": TS,
                "attempt": attempt,
                "text": text.hex(),
                "plain": plain.hex(),
                "ack": crc,
            }
        )
        return crc

    crc_flood = add_txt("txt_flood", "alice", "bob", 0, b"Hello Bob")
    add_txt(
        "txt_attempt5",
        "alice",
        "bob",
        5,
        bytes.fromhex("53616c75742c20c3a761207661203f"),
    )
    add_txt(
        "txt_direct",
        "alice",
        "bob",
        1,
        b"par le chemin direct",
        route=2,
        path=bytes.fromhex("abcd"),
        path_len=2,
    )
    add_txt(
        "txt_cli_data",
        "alice",
        "bob",
        0,
        b"CLI output",
        route=1,
    )

    # Attempt-number boundaries. The ACK CRC hashes ts || (txt_type<<2 | attempt&3)
    # || text and never the NUL + attempt tail that attempts > 3 append
    # (BaseChatMesh::composeMsgPacket), so attempt 4 has the same CRC as attempt 0
    # even though its ciphertext differs. attempt&3 wraps, so 255 behaves like 3.
    crc_attempt3 = add_txt("txt_attempt3", "alice", "bob", 3, b"Hello Bob")
    crc_attempt4 = add_txt("txt_attempt4", "alice", "bob", 4, b"Hello Bob")
    crc_attempt255 = add_txt("txt_attempt255", "alice", "bob", 255, b"Hello Bob")
    assert crc_attempt4 == crc_flood, "attempt 4 must reuse attempt 0's ACK CRC"
    assert crc_attempt255 == crc_attempt3, "attempt 255 must reuse attempt 3's ACK CRC"
    assert crc_attempt3 != crc_flood, "attempts 0 and 3 must differ"

    # AES-128 block edges. The plaintext is 5 + len bytes (+2 once attempt > 3),
    # zero-padded to 16, so these land exactly on a block and one byte over it.
    add_txt("txt_block_exact", "alice", "bob", 0, b"x" * 11)  # 16 bytes
    add_txt("txt_block_over", "alice", "bob", 0, b"x" * 12)  # 17 bytes
    add_txt("txt_attempt4_block_exact", "alice", "bob", 4, b"y" * 9)  # 5 + 9 + 2 = 16
    add_txt("txt_attempt4_block_over", "alice", "bob", 4, b"y" * 10)  # 17

    ack_extra = ack_payload(crc_flood, 0, 0x5A)
    add_packet(v, "ack_direct", 2, 3, ack_extra, b"\xA1", 1)
    v["ack"].append(
        {
            "packet": "ack_direct",
            "crc": crc_flood,
            "extended_attempt": 0,
            "random": 0x5A,
        }
    )

    path_ack_plain = path_plain(0x42, bytes.fromhex("12345678"), 3, ack_extra)
    path_ack_payload = addressed_payload(
        ids["alice"]["pub"][0], ids["bob"]["pub"][0], secret_ab, path_ack_plain
    )
    add_packet(v, "path_return_ack", 1, 8, path_ack_payload)
    v["path"].append(
        {
            "packet": "path_return_ack",
            "in_path_len": 0x42,
            "in_path": "12345678",
            "extra_type": 3,
            "extra": ack_extra.hex(),
            "plain": path_ack_plain.hex(),
        }
    )

    path_none_plain = path_plain(0, b"", 0, b"", bytes.fromhex("11223344"))
    path_none_payload = addressed_payload(
        ids["alice"]["pub"][0], ids["bob"]["pub"][0], secret_ab, path_none_plain
    )
    add_packet(v, "path_return_none", 1, 8, path_none_payload)
    v["path"].append(
        {
            "packet": "path_return_none",
            "in_path_len": 0,
            "in_path": "",
            "extra_type": 0xFF,
            "extra": "11223344",
            "plain": path_none_plain.hex(),
        }
    )

    grp_plain = group_txt_plain(TS, b"Alice", b"Bonjour \xc3\xa0 tous")
    grp_payload = group_payload(channel_hash(public), public, grp_plain)
    add_packet(v, "grp_txt_public", 1, 5, grp_payload)
    v["group"].append(
        {
            "packet": "grp_txt_public",
            "secret": public.hex(),
            "channel_hash": channel_hash(public),
            "plain": grp_plain.hex(),
            "subtype": "txt",
        }
    )

    grp_fr_plain = group_txt_plain(TS, b"Alice", b"dans la portee fr")
    grp_fr_payload = group_payload(channel_hash(public), public, grp_fr_plain)
    tc_fr = transport_code(region_key("fr"), 5, grp_fr_payload)
    add_packet(
        v, "grp_txt_public_fr", 0, 5, grp_fr_payload, transport=(tc_fr, 0)
    )
    v["group"].append(
        {
            "packet": "grp_txt_public_fr",
            "secret": public.hex(),
            "channel_hash": channel_hash(public),
            "plain": grp_fr_plain.hex(),
            "subtype": "txt",
            "region": "fr",
            "region_key": region_key("fr").hex(),
        }
    )

    data = bytes(range(0x64, 0x78))
    grp_data_plain_bytes = group_data_plain(0xFF4C, data)
    grp_data_payload = group_payload(
        channel_hash(custom), custom, grp_data_plain_bytes
    )
    add_packet(v, "grp_data_test", 1, 6, grp_data_payload)
    v["group"].append(
        {
            "packet": "grp_data_test",
            "secret": custom.hex(),
            "channel_hash": channel_hash(custom),
            "plain": grp_data_plain_bytes.hex(),
            "subtype": "data",
            "data_type": 0xFF4C,
            "data": data.hex(),
        }
    )

    anon_plain = le32(TS) + b"password"
    anon_secret = secret_ar
    anon = anon_payload(
        ids["relay"]["pub"][0], ids["alice"]["pub"], anon_secret, anon_plain
    )
    add_packet(v, "anon_login", 1, 7, anon)
    v["anon"].append(
        {
            "packet": "anon_login",
            "from": "alice",
            "to": "relay",
            "secret": anon_secret.hex(),
            "plain": anon_plain.hex(),
        }
    )

    req_plain = le32(0x11223344) + bytes([1]) + b"\x00\x00\x00\x00" + bytes.fromhex(
        "aabbccdd"
    )
    req = addressed_payload(
        ids["relay"]["pub"][0], ids["alice"]["pub"][0], secret_ar, req_plain
    )
    add_packet(v, "req_generic", 1, 0, req)
    v["req"].append(
        {
            "packet": "req_generic",
            "from": "alice",
            "to": "relay",
            "tag": 0x11223344,
            "req_type": 1,
            "plain": req_plain.hex(),
        }
    )

    ka_plain = le32(TS) + bytes([2]) + le32(TS - 60)
    ka = addressed_payload(
        ids["relay"]["pub"][0], ids["alice"]["pub"][0], secret_ar, ka_plain
    )
    add_packet(v, "req_keepalive", 1, 0, ka)
    v["req"].append(
        {
            "packet": "req_keepalive",
            "from": "alice",
            "to": "relay",
            "tag": TS,
            "req_type": 2,
            "plain": ka_plain.hex(),
        }
    )

    resp_plain = (
        le32(TS + 10)
        + b"\x00\x00"
        + bytes([1, 0x0F])
        + bytes.fromhex("01020304")
        + le16(2)
    )
    resp = addressed_payload(
        ids["alice"]["pub"][0], ids["relay"]["pub"][0], secret_ar, resp_plain
    )
    add_packet(v, "response_login", 1, 1, resp)
    v["response"].append(
        {
            "packet": "response_login",
            "from": "relay",
            "to": "alice",
            "tag": TS + 10,
            "plain": resp_plain.hex(),
        }
    )

    trace_payload = (
        le32(0x01020304) + le32(0xA0B0C0D0) + bytes([0]) + bytes.fromhex("112233")
    )
    add_packet(v, "trace_one_snr", 2, 9, trace_payload, b"\x28", 1)
    v["trace"].append(
        {
            "packet": "trace_one_snr",
            "tag": 0x01020304,
            "auth": 0xA0B0C0D0,
            "flags": 0,
            "hashes": "112233",
        }
    )

    multipart_payload = bytes([0x13]) + le32(crc_flood)
    add_packet(v, "multipart_ack", 2, 10, multipart_payload)
    v["multipart"].append(
        {
            "packet": "multipart_ack",
            "remaining": 1,
            "inner_type": 3,
            "body": le32(crc_flood).hex(),
        }
    )

    control_payload = bytes([0x80, 0x04]) + bytes.fromhex("efbeadde")
    add_packet(v, "control_discover", 2, 11, control_payload)
    v["control"].append(
        {"packet": "control_discover", "flags": 0x80, "data": "04efbeadde"}
    )

    control_resp_payload = (
        bytes([0x92, (10 * 4) & 0xFF])
        + bytes.fromhex("efbeadde")
        + ids["relay"]["pub"][:8]
    )
    add_packet(v, "control_discover_resp", 2, 11, control_resp_payload)
    v["control"].append(
        {
            "packet": "control_discover_resp",
            "flags": 0x92,
            "data": control_resp_payload[1:].hex(),
        }
    )

    raw_payload = bytes.fromhex("deadbeef")
    add_packet(v, "raw_custom", 2, 15, raw_payload)
    v["raw"].append({"packet": "raw_custom", "payload": raw_payload.hex()})

    for region in ("fr", "#fr", "be"):
        key = region_key(region)
        for ptype, payload in (
            (5, bytes.fromhex("010203")),
            (4, bytes(range(0x64))),
        ):
            v["transport"].append(
                {
                    "region": region,
                    "ptype": ptype,
                    "payload": payload.hex(),
                    "code": transport_code(key, ptype, payload),
                }
            )

    for name, raw in REJECTS:
        v["reject"].append({"name": name, "raw": raw})

    return v


# ---------------------------------------------------------------------------------------------
# Emission

def quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def emit(data: dict) -> str:
    order = [
        "identity",
        "ecdh",
        "seal",
        "channel",
        "packet",
        "advert",
        "txt",
        "ack",
        "group",
        "path",
        "anon",
        "req",
        "response",
        "trace",
        "multipart",
        "control",
        "raw",
        "transport",
        "reject",
    ]
    lines = [
        "# Generated by tools/oracle/gen.py. Do not edit by hand.",
        "# Independent oracle: MeshCore composition + cryptography/PyNaCl primitives.",
        "# See tools/oracle/README.md.",
        "",
    ]
    for section in order:
        for entry in data.get(section, []):
            lines.append(f"[[{section}]]")
            for key, value in entry.items():
                if isinstance(value, str):
                    lines.append(f"{key} = {quote(value)}")
                elif isinstance(value, list):
                    lines.append(f"{key} = [{', '.join(str(x) for x in value)}]")
                else:
                    lines.append(f"{key} = {value}")
            lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--emit",
        action="store_true",
        help="write the oracle vector TOML",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="write --emit output to this path (default: stdout)",
    )
    args = parser.parse_args()

    for name in selfcheck.run():
        print(f"selfcheck  {name}  OK", file=sys.stderr)

    data = build()

    if args.emit:
        text = emit(data)
        if args.output:
            args.output.write_text(text)
            print(f"wrote {args.output}", file=sys.stderr)
        else:
            sys.stdout.write(text)
        return 0

    sections = [
        "identity",
        "ecdh",
        "seal",
        "channel",
        "packet",
        "advert",
        "txt",
        "ack",
        "group",
        "path",
        "anon",
        "req",
        "response",
        "trace",
        "multipart",
        "control",
        "raw",
        "transport",
        "reject",
    ]
    total = 0
    print("vector set:", file=sys.stderr)
    for section in sections:
        count = len(data.get(section, []))
        if count:
            total += count
            print(f"  {section:<10} {count:>3}", file=sys.stderr)
    print(f"  {'total':<10} {total:>3}", file=sys.stderr)
    print(
        f"\nwrote {len(emit(data))} bytes of deterministic vectors; "
        "regenerate with --emit --output",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
