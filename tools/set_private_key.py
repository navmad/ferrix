#!/usr/bin/env python3
"""Set the MeshCore private key over the USB companion protocol.

This bypasses the text CLI's line-length limits by sending the key as a
binary CMD_IMPORT_PRIVATE_KEY frame.

Usage:
    python3 tools/set_private_key.py /dev/cu.usbmodem1101 <128 hex chars>

The key is the 64-byte expanded private key (128 hex characters).
"""

import os
import re
import select
import sys
import termios
import time

CMD_IMPORT_PRIVATE_KEY = 0x18
CMD_APP_START = 0x01


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag &= ~(
        termios.IGNBRK
        | termios.BRKINT
        | termios.PARMRK
        | termios.ISTRIP
        | termios.INLCR
        | termios.IGNCR
        | termios.ICRNL
        | termios.IXON
    )
    oflag &= ~termios.OPOST
    lflag &= ~(
        termios.ECHO
        | termios.ECHONL
        | termios.ICANON
        | termios.ISIG
        | termios.IEXTEN
    )
    cflag &= ~(termios.CSIZE | termios.PARENB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(
        fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]
    )
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_frame(fd, timeout=2.0):
    """Return the first complete binary frame, or None."""
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], 0.1)
        if not ready:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if not chunk:
            break
        buf += chunk
        start = buf.find(b">")
        while start >= 0 and len(buf) >= start + 3:
            length = int.from_bytes(buf[start + 1 : start + 3], "little")
            end = start + 3 + length
            if len(buf) >= end:
                return buf[start + 3 : end]
            break
        buf = buf[start:] if start >= 0 else b""
    return None


def send_frame(fd, payload):
    os.write(fd, b"<" + len(payload).to_bytes(2, "little") + bytes(payload))


def normalize_key(value):
    value = value.strip()
    if value.lower().startswith("0x"):
        value = value[2:]
    value = re.sub(r"[\s:_\-'\"]", "", value)
    if len(value) != 128 or not all(c in "0123456789abcdefABCDEF" for c in value):
        raise SystemExit("error: expected exactly 128 hex characters")
    return bytes.fromhex(value)


def main():
    if len(sys.argv) < 3:
        raise SystemExit(
            "usage: python3 tools/set_private_key.py <port> <128 hex chars>"
        )
    port, raw_key = sys.argv[1], sys.argv[2]
    key = normalize_key(raw_key)

    fd = open_port(port)
    try:
        send_frame(fd, [CMD_IMPORT_PRIVATE_KEY] + list(key))
        response = read_frame(fd)
        if response is None:
            raise SystemExit("error: no response from device")
        if response and response[0] == 0:
            print("OK: private key accepted")
        else:
            raise SystemExit(f"error: import failed (response {response.hex()})")

        # Verify the derived public key.
        send_frame(fd, [CMD_APP_START])
        info = read_frame(fd)
        if info and info[0] == 5 and len(info) >= 36:
            print("public key:", info[4:36].hex())
        else:
            print("warning: could not read SELF_INFO to verify the public key")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
