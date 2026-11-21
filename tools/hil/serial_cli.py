"""Minimal USB-CDC helper for talking to a Ferrix board (stdlib only).

Two protocols share the port:
  * the text CLI: ``cmd\\r\\n`` in, echo + ``  -> reply`` lines out;
  * MeshCore companion frames: ``<`` + u16le len + payload in,
    ``>`` + u16le len + payload out (see crates/ferrix-core/src/companion/framing.rs).
"""
import glob
import json
import os
import select
import sys
import termios
import time

PORT_GLOBS = ("/dev/cu.usbmodem*", "/dev/ttyACM*")
MAX_FRAME_PAYLOAD = 256


def candidate_ports():
    found = []
    for pattern in PORT_GLOBS:
        found.extend(glob.glob(pattern))
    return sorted(set(found))


def open_raw(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
               | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG
               | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def frame(payload):
    """Wrap a companion command payload for the USB transport."""
    if len(payload) > MAX_FRAME_PAYLOAD:
        raise ValueError("payload too long")
    return b"<" + len(payload).to_bytes(2, "little") + bytes(payload)


def parse_frame(buf):
    """Return (payload, rest) for the first complete ``>`` frame, or (None, buf)."""
    i = 0
    while True:
        i = buf.find(b">", i)
        if i < 0 or len(buf) - i < 3:
            return None, buf if i < 0 else buf[i:]
        n = int.from_bytes(buf[i + 1:i + 3], "little")
        if n > MAX_FRAME_PAYLOAD:
            i += 1  # not a frame header: text such as "> 910.525,..."
            continue
        if len(buf) - i - 3 < n:
            return None, buf[i:]
        return bytes(buf[i + 3:i + 3 + n]), buf[i + 3 + n:]


def clean_lines(raw, cmd=None):
    """Decode CLI output into reply lines without echo or ``  -> `` prompts."""
    out = []
    for line in raw.decode("utf-8", "replace").replace("\r", "\n").split("\n"):
        line = line.rstrip()
        if not line.strip():
            continue
        if cmd is not None and line.strip() == cmd.strip():
            continue  # interactive echo
        if line.startswith("  -> "):
            line = line[5:]
        elif line.startswith("  ->"):
            line = line[4:]
        if line.startswith("> "):
            line = line[2:]
        out.append(line.strip())
    return out


class SerialDevice:
    """One open CDC port speaking the CLI and companion framing."""

    def __init__(self, path):
        self.path = path
        self.fd = open_raw(path)
        self.pending = b""

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    # -- raw I/O -----------------------------------------------------------
    def _read(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 512)
        except BlockingIOError:
            return b""

    def drain(self, quiet=0.15):
        end = time.time() + quiet
        while time.time() < end:
            if self._read(0.05):
                end = time.time() + quiet
        self.pending = b""

    def write(self, data):
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
            except BlockingIOError:
                select.select([], [self.fd], [], 0.1)
                continue
            view = view[n:]

    # -- text CLI ----------------------------------------------------------
    def cmd(self, command, timeout=2.0, quiet=0.3):
        """Send one CLI line and return its cleaned reply lines."""
        self.drain(0.05)
        self.write((command + "\r\n").encode())
        buf = b""
        start = time.time()
        last = None
        while time.time() - start < timeout:
            chunk = self._read(0.05)
            if chunk:
                buf += chunk
                last = time.time()
            elif last is not None and time.time() - last >= quiet:
                break
        return clean_lines(buf, command)

    def json(self, command, timeout=2.0):
        lines = self.cmd(command, timeout)
        for line in lines:
            if line.startswith("{"):
                return json.loads(line)
        raise RuntimeError("%r: no JSON reply in %r" % (command, lines))

    # -- companion frames --------------------------------------------------
    def exchange(self, payload, timeout=3.0):
        """Send one companion command and return the first reply payload."""
        self.drain(0.1)
        self.write(frame(payload))
        buf = b""
        end = time.time() + timeout
        while time.time() < end:
            buf += self._read(0.05)
            reply, buf = parse_frame(buf)
            if reply is not None:
                return reply
        raise TimeoutError("no companion frame within %.1fs" % timeout)

    # -- lifecycle ---------------------------------------------------------
    def reboot_and_reconnect(self, expect, timeout=40.0):
        """Send `reboot`, wait for the port to drop and a matching board to return."""
        old = self.path
        self.write(b"reboot\r\n")
        time.sleep(0.5)
        self.close()
        end = time.time() + timeout
        gone = False
        while time.time() < end:
            if not os.path.exists(old):
                gone = True
            for path in candidate_ports():
                if not gone and path == old:
                    continue  # still the pre-reboot node
                if probe(path, expect):
                    self.path = path
                    self.fd = open_raw(path)
                    self.pending = b""
                    return
            time.sleep(0.5)
        raise TimeoutError("board did not come back within %.0fs" % timeout)


def probe(path, expect):
    """True if `path` is a Ferrix CLI whose `board` line contains `expect`."""
    try:
        dev = SerialDevice(path)
    except OSError:
        return False
    try:
        time.sleep(0.2)
        return any(expect in line for line in dev.cmd("board", timeout=1.5))
    except OSError:
        return False
    finally:
        dev.close()


def find_device(expect, explicit=None):
    """Return the port path of the board whose `board` reply contains `expect`."""
    paths = [explicit] if explicit else candidate_ports()
    for path in paths:
        if probe(path, expect):
            return path
    return None


if __name__ == "__main__":
    print("\n".join(candidate_ports()) or "no candidate ports", file=sys.stderr)
