#!/usr/bin/env python3
"""Known-answer self-checks for the oracle's primitives.

The oracle refuses to be trusted unless every published known-answer test
here passes. This catches a broken dependency or a wrong wrapper before any
MeshCore vector is composed.
"""

import hashlib
import hmac

import nacl.bindings as nb

from primitives import aes_ecb_decrypt, aes_ecb_encrypt, ed25519_pub, ed25519_sign, expand


def run() -> list[str]:
    passed: list[str] = []

    # AES-128, FIPS-197 Appendix C.1.
    key = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    plain = bytes.fromhex("00112233445566778899aabbccddeeff")
    cipher = bytes.fromhex("69c4e0d86a7b0430d8cdb78070b4c55a")
    assert aes_ecb_encrypt(key, plain) == cipher, "AES-128 FIPS-197 encrypt"
    assert aes_ecb_decrypt(key, cipher) == plain, "AES-128 FIPS-197 decrypt"
    passed.append("AES-128 FIPS-197")

    # SHA-256.
    assert (
        hashlib.sha256(b"abc").hexdigest()
        == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
    ), "SHA-256"
    passed.append("SHA-256")

    # HMAC-SHA256, RFC 4231 test case 1.
    assert (
        hmac.new(b"\x0b" * 20, b"Hi There", hashlib.sha256).hexdigest()
        == "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
    ), "HMAC-SHA256 RFC 4231"
    passed.append("HMAC-SHA256 RFC 4231")

    # Ed25519, RFC 8032 test 1.
    seed = bytes.fromhex(
        "9d61b19deffd5a60ba844af492ec2cc44449c5697b326919703bac031cae7f60"
    )
    pub = bytes.fromhex(
        "d75a980182b10ab7d54bfed3c964073a0ee172f3daa62325af021a68f707511a"
    )
    sig = bytes.fromhex(
        "e5564300c360ac729086e2cc806e828a84877f1eb8e5d974d873e06522490155"
        "5fb8821590a33bacc61e39701cf9b46bd25bf5f0595bbe24655141438e7a100b"
    )
    assert ed25519_pub(seed) == pub, "Ed25519 public key RFC 8032"
    assert ed25519_sign(seed, b"") == sig, "Ed25519 signature RFC 8032"
    passed.append("Ed25519 RFC 8032")

    # X25519, RFC 7748 section 5.2.
    scalar = bytes.fromhex(
        "a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4"
    )
    u = bytes.fromhex(
        "e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c"
    )
    want = bytes.fromhex(
        "c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552"
    )
    assert nb.crypto_scalarmult(scalar, u) == want, "X25519 RFC 7748"
    passed.append("X25519 RFC 7748")

    # Ed25519 expanded key -> public key, and Ed25519 -> Montgomery symmetry.
    seed_a = bytes.fromhex(
        "b6ff817b2d5b3108fff31ef8c61c3207a08d098a12e2bc694c680abf276f6b6c"
    )
    seed_b = bytes.fromhex(
        "8878fd3eeb1fd3a57d2006c66e3a3f33fbd5214ed03d2acb077cc014b46a60ea"
    )
    prv_a, prv_b = expand(seed_a), expand(seed_b)
    pub_a, pub_b = ed25519_pub(seed_a), ed25519_pub(seed_b)
    assert nb.crypto_scalarmult_ed25519_base_noclamp(prv_a[:32]) == pub_a
    assert nb.crypto_scalarmult_ed25519_base_noclamp(prv_b[:32]) == pub_b
    u_a = nb.crypto_sign_ed25519_pk_to_curve25519(pub_a)
    u_b = nb.crypto_sign_ed25519_pk_to_curve25519(pub_b)
    assert nb.crypto_scalarmult(prv_a[:32], u_b) == nb.crypto_scalarmult(prv_b[:32], u_a)
    passed.append("Ed25519 expanded key + X25519 symmetry")

    return passed


if __name__ == "__main__":
    for name in run():
        print(f"selfcheck  {name}  OK")
