# MeshCore vector oracle

The generator for MeshCore over-the-air conformance vectors.

The point is independence: this code must never import `meshcore-wire` or call
the Rust implementation. It composes the bytes from MeshCore's rules and
delegates the cryptography to audited libraries:

- AES-128-ECB and standard Ed25519: `cryptography`
- Ed25519 point operations and X25519: `PyNaCl` / libsodium
- SHA-256/512 and HMAC: Python standard library

Current scope:

- identity: seed -> expanded 64-byte key -> public key
- ECDH shared secret (both directions)
- `seal` / `open`: `MAC(2) || AES-128-ECB(zero-padded plaintext)`
- channel secrets, channel hashes and hashtag channels
- `ADVERT` (kind, location, feat1/feat2, name, signature)
- `TXT_MSG` (plain and extended-attempt), `ACK` and `PATH`
- `GRP_TXT`, `GRP_DATA`, `ANON_REQ`, `REQ`, `RESPONSE`
- `TRACE`, `MULTIPART`, `CONTROL`, `RAW_CUSTOM`
- transport codes and transport-scoped packets
- packet framing for flood, direct and transport routes

The generated file is `crates/meshcore-wire/testdata/oracle_vectors.toml`. It
is the vector source for the Rust conformance tests; there is no second copy
to keep in sync.

## Setup

```sh
uv venv tools/oracle/.venv
uv pip install --python tools/oracle/.venv/bin/python -r tools/oracle/requirements.txt
```

## Run

```sh
# self-check primitives and print the vector summary
tools/oracle/.venv/bin/python tools/oracle/gen.py

# regenerate the checked-in oracle vector file
tools/oracle/.venv/bin/python tools/oracle/gen.py \
    --emit --output crates/meshcore-wire/testdata/oracle_vectors.toml
```

Every primitive is first checked against a published known-answer test
(FIPS-197 AES, RFC 8032 Ed25519, RFC 7748 X25519, RFC 4231 HMAC). A mismatch
aborts before anything is emitted.

The output is deterministic; `--emit` writes the same bytes every time. CI
runs the self-check and a regeneration diff, so the checked-in file cannot
drift.

## Next steps

1. Add seeded random valid packets and more rejection/adversarial cases.
2. Add real captures from MeshCore on the M1 as an additional validation set.
