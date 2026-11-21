#!/usr/bin/env python3
"""Hardware-in-the-loop smoke test for the Seeed Wio Tracker L1 Pro.

Talks to a connected board over its USB CDC port (text CLI + companion
frames) and verifies the things host tests cannot: clocks, regulators, QSPI
flash, GPS UART, battery ADC, config persistence across a real reboot and the
binary companion handshake.

    ./tools/build.sh wio-tracker-l1-pro hil [options]

Options:
  --port PATH         serial port (default: auto-detect by `board` reply)
  --internal-storage  firmware was built with --features internal-storage
  --gps-timeout S     seconds to wait for GPS UART traffic (default 30)
  --tx                also transmit one zero-hop advert and expect the TX
                      counter to move (RF is emitted on the configured channel)
  --skip-reboot       skip the reboot/persistence check
  --soak S            keep polling the CLI for S seconds and check stability
  --flash             build + flash release firmware first (tools/flash_serial.sh)
  --json PATH         also write the results as JSON
  --self-test         run the script's own logic against a scripted fake
                      device (no hardware needed)

Exit status: 0 all checks passed (skips allowed), 1 a check failed,
2 bad usage, 3 no Wio Tracker L1 Pro found.
"""
import argparse
import json
import os
import random
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial_cli  # noqa: E402

BOARD_NAME = "Wio Tracker L1 Pro"
P25Q16H_ID = "856015"  # platforms/nrf52/src/qspi.rs
BANDWIDTHS = {7.8, 10.4, 15.6, 20.8, 31.25, 41.67, 62.5, 125.0, 250.0, 500.0}
CMD_APP_START = 1
CMD_DEVICE_QUERY = 22
RESP_SELF_INFO = 5
RESP_DEVICE_INFO = 13


class Skip(Exception):
    """A check that cannot run in this configuration."""


# --- parsers (unit-tested by --self-test) -----------------------------------

def parse_radio(line):
    """`910.525,62.5,7,5` -> (freq_mhz, bw_khz, sf, cr)."""
    m = re.fullmatch(r"(\d+\.\d+),(\d+(?:\.\d+)?),(\d+),(\d+)", line.strip())
    if not m:
        raise AssertionError("unparseable radio line %r" % line)
    return float(m[1]), float(m[2]), int(m[3]), int(m[4])


def parse_battery(line):
    """`3987 mV (85%)` -> (mv, pct)."""
    m = re.fullmatch(r"(\d+) mV \((\d+)%\)", line.strip())
    if not m:
        raise AssertionError("unparseable battery line %r" % line)
    return int(m[1]), int(m[2])


def first_line(lines, pattern):
    for line in lines:
        if re.search(pattern, line):
            return line
    raise AssertionError("no line matching /%s/ in %r" % (pattern, lines))


# --- checks -----------------------------------------------------------------

class Ctx:
    def __init__(self, dev, args):
        self.dev = dev
        self.args = args


def check_identity(c):
    ver = first_line(c.dev.cmd("ver"), r"^\d+\.\d+\.\d+\S*-ferrix$")
    board = first_line(c.dev.cmd("board"), r"^" + re.escape(BOARD_NAME))
    return "%s on %s" % (ver, board)


def check_power(c):
    j = c.dev.json("stats-power")
    assert j.get("lfclk_running") is True, "LFCLK not running: %s" % j
    assert j.get("lfclk_src") == "xtal", "LFCLK source is %r, expected xtal (LFXO)" % j.get("lfclk_src")
    assert str(j.get("lfclk_cfg", "")).startswith("xtal"), "LFCLK configured as %r" % j.get("lfclk_cfg")
    assert j.get("dcdc_reg1") is True, "REG1 DC/DC not active: %s" % j
    return "LFXO running, REG1 DC/DC on"


def check_storage(c):
    flash = c.dev.json("stats-flash")
    if c.args.internal_storage:
        assert flash.get("ext") == "internal", "stats-flash ext=%r" % flash.get("ext")
        return "internal-storage profile"
    q = c.dev.json("stats-qspi")
    assert q.get("status") == "ready", "QSPI status %r" % q.get("status")
    assert q.get("id") == P25Q16H_ID, "JEDEC id %r, expected %s (P25Q16H)" % (q.get("id"), P25Q16H_ID)
    assert flash.get("ext") == "ready", "stats-flash ext=%r" % flash.get("ext")
    return "P25Q16H %s ready" % q["id"]


def check_radio_config(c):
    mhz, bw, sf, cr = parse_radio(first_line(c.dev.cmd("get radio"), r"^\d+\.\d+,"))
    assert 150.0 <= mhz <= 960.0, "frequency %s MHz out of range" % mhz
    assert bw in BANDWIDTHS, "bandwidth %s kHz is not a LoRa bandwidth" % bw
    assert 5 <= sf <= 12, "SF%d" % sf
    assert 5 <= cr <= 8, "CR4/%d" % cr
    stats = c.dev.json("stats-radio")
    assert "error" not in stats and "tx_sent" in stats, "stats-radio: %s" % stats
    return "%.3f MHz BW%g SF%d CR4/%d" % (mhz, bw, sf, cr)


def check_battery(c):
    mv, pct = parse_battery(first_line(c.dev.cmd("battery"), r"mV"))
    assert 3000 <= mv <= 4500, "battery %d mV is implausible" % mv
    assert 0 <= pct <= 100, "battery %d%%" % pct
    core = c.dev.json("stats-core")
    assert abs(core["battery_mv"] - mv) <= 150, "stats-core %s mV vs battery %s mV" % (core["battery_mv"], mv)
    return "%d mV (%d%%)" % (mv, pct)


def check_companion_handshake(c):
    info = c.dev.exchange(bytes([CMD_DEVICE_QUERY, 3]))
    assert info[0] == RESP_DEVICE_INFO, "device query answered with code %d" % info[0]
    assert info[1] >= 10, "firmware_ver %d is too old for path_hash_mode" % info[1]
    semver = info[60:80].split(b"\0")[0].decode("ascii", "replace")
    assert re.fullmatch(r"v\d+\.\d+\.\d+", semver), "protocol semver %r" % semver
    # The model string intentionally stays "Elecrow ThinkNode M1" (drop-in
    # compatibility with MeshCore apps), so it is not asserted here.

    me = c.dev.exchange(bytes([CMD_APP_START, 3]) + b"\0" * 6 + b"ferrix-hil")
    assert me[0] == RESP_SELF_INFO, "app start answered with code %d" % me[0]
    pubkey = me[4:36].hex()
    freq_khz = int.from_bytes(me[48:52], "little")
    name = me[58:].decode("utf-8", "replace")

    cli_key = first_line(c.dev.cmd("get public.key"), r"^[0-9a-f]{64}$")
    assert pubkey == cli_key, "self-info key %s != CLI key %s" % (pubkey, cli_key)
    cli_mhz = parse_radio(first_line(c.dev.cmd("get radio"), r"^\d+\.\d+,"))[0]
    assert abs(freq_khz / 1000.0 - cli_mhz) < 0.002, "self-info %d kHz vs CLI %.3f MHz" % (freq_khz, cli_mhz)
    cli_name = first_line(c.dev.cmd("get name"), r".+")
    assert name == cli_name, "self-info name %r != CLI name %r" % (name, cli_name)
    return "%s, protocol %s, node %r" % (me[4:8].hex(), semver, name)


def check_gps(c):
    end = time.time() + c.args.gps_timeout
    last = None
    while time.time() < end:
        last = c.dev.json("stats-gps")
        if not last.get("power"):
            raise Skip("GPS is powered off (enable it on the Position page)")
        if last.get("bytes", 0) > 0:
            return "%d bytes, %d lines, fix=%s sats=%s" % (
                last["bytes"], last.get("lines", 0), last.get("fix"), last.get("sats"))
        time.sleep(2.0)
    raise AssertionError("no NMEA bytes from the L76K in %.0fs (UART pins / enable polarity?): %s"
                         % (c.args.gps_timeout, last))


def check_tx(c):
    if not c.args.tx:
        raise Skip("pass --tx to transmit a zero-hop advert")
    before = c.dev.json("stats-packets")["sent"]
    c.dev.cmd("advert.zerohop")
    end = time.time() + 15
    while time.time() < end:
        now = c.dev.json("stats-packets")["sent"]
        if now > before:
            return "sent counter %d -> %d" % (before, now)
        time.sleep(1.0)
    raise AssertionError("advert was not transmitted (SX1262 / RF path?)")


def check_persistence(c):
    if c.args.skip_reboot:
        raise Skip("--skip-reboot")
    original = first_line(c.dev.cmd("get name"), r".+")
    probe = "hil%04d" % random.randrange(10000)
    try:
        first_line(c.dev.cmd("set name " + probe), r"OK|ok|Name|name")
        first_line(c.dev.cmd("save"), r"OK")
        c.dev.reboot_and_reconnect(BOARD_NAME)
        got = first_line(c.dev.cmd("get name"), r".+")
        assert got == probe, "name after reboot is %r, expected %r" % (got, probe)
    finally:
        try:
            c.dev.cmd("set name " + original)
            c.dev.cmd("save")
        except Exception as e:  # best effort: leave the board as found
            print("warning: could not restore name %r: %s" % (original, e), file=sys.stderr)
    return "name survived reboot; restored %r" % original


def check_soak(c):
    if not c.args.soak:
        raise Skip("pass --soak SECONDS")
    start = c.dev.json("stats-wdt")
    end = time.time() + c.args.soak
    polls = misses = 0
    while time.time() < end:
        polls += 1
        try:
            first_line(c.dev.cmd("ver", timeout=1.5), r"-ferrix$")
        except Exception:
            misses += 1
        time.sleep(1.0)
    final = c.dev.json("stats-wdt")
    assert misses == 0, "%d/%d CLI polls went unanswered" % (misses, polls)
    assert final["dog"] is False, "last reset was a watchdog reset: %s" % final
    assert final["pets"] > start["pets"], "watchdog stopped being fed: %s -> %s" % (start, final)
    assert final["app"] > start["app"], "app loop heartbeat stalled"
    return "%d polls, no misses, watchdog fed" % polls


CHECKS = [
    ("identity", check_identity),
    ("power", check_power),
    ("storage", check_storage),
    ("radio_config", check_radio_config),
    ("battery", check_battery),
    ("companion_handshake", check_companion_handshake),
    ("gps_uart", check_gps),
    ("tx", check_tx),
    ("persistence", check_persistence),
    ("soak", check_soak),
]


def run_checks(dev, args, out=sys.stdout):
    ctx = Ctx(dev, args)
    results = []
    for name, fn in CHECKS:
        t0 = time.time()
        try:
            detail, status = fn(ctx), "PASS"
        except Skip as e:
            detail, status = str(e), "SKIP"
        except Exception as e:  # AssertionError, timeouts, JSON/IO errors
            detail, status = "%s: %s" % (type(e).__name__, e), "FAIL"
        results.append({"name": name, "status": status, "detail": detail,
                        "seconds": round(time.time() - t0, 2)})
        print("%-4s  %-20s %s" % (status, name, detail), file=out)
        if name == "identity" and status == "FAIL":
            print("      not a Wio Tracker L1 Pro; aborting", file=out)
            break
    return results


# --- self-test ---------------------------------------------------------------

class FakeDevice:
    """Replies exactly like the firmware's CLI/companion handlers (cleaned)."""

    path = "fake"

    def __init__(self, **overrides):
        self.name = self.saved_name = "orig"
        self.sent = 3
        self.gps_bytes = 0
        self.pets = 100
        self.app = 10
        self.o = overrides
        self.key = bytes(range(32))

    def cmd(self, c, timeout=2.0):
        if c == "ver":
            return [self.o.get("ver", "1.18.2-ferrix")]
        if c == "board":
            return [self.o.get("board", BOARD_NAME + " (nRF52840, SX1262 LoRa)")]
        if c == "get radio":
            return ["910.525,62.5,7,5"]
        if c == "battery":
            return [self.o.get("battery", "3987 mV (85%)")]
        if c == "get public.key":
            return [self.key.hex()]
        if c == "get name":
            return [self.name]
        if c.startswith("set name "):
            self.name = c[9:]
            return ["OK"]
        if c == "save":
            self.saved_name = self.name
            return ["OK (Config saved to flash)"]
        if c == "advert.zerohop":
            self.sent += 1
            return ["OK"]
        raise AssertionError("fake: unhandled command %r" % c)

    def json(self, c, timeout=2.0):
        if c == "stats-power":
            return {"lfclk_cfg": "xtal(20ppm)", "lfclk_running": True,
                    "lfclk_src": self.o.get("lfclk_src", "xtal"), "lfclkstat": "0x00010001",
                    "dcdc_reg1_cfg": True, "dcdc_reg1": True, "dcdc_reg0": False}
        if c == "stats-qspi":
            return {"status": "ready", "id": self.o.get("qspi_id", P25Q16H_ID)}
        if c == "stats-flash":
            return {"ext": "internal" if self.o.get("internal") else "ready"}
        if c == "stats-radio":
            return {"noise_floor": -118, "tx_sent": self.sent}
        if c == "stats-packets":
            return {"recv": 0, "sent": self.sent}
        if c == "stats-core":
            return {"battery_mv": 3990, "battery_pct": 85}
        if c == "stats-gps":
            self.gps_bytes += 0 if self.o.get("gps_dead") else 200
            return {"power": True, "fix": False, "sats": 0, "bytes": self.gps_bytes, "lines": self.gps_bytes // 40}
        if c == "stats-wdt":
            self.pets += 5
            self.app += 5
            return {"app": self.app, "storage": 3, "pets": self.pets,
                    "reset": "0x00000004", "dog": False}
        raise AssertionError("fake: unhandled json command %r" % c)

    def exchange(self, payload, timeout=3.0):
        if payload[0] == CMD_DEVICE_QUERY:
            r = bytes([RESP_DEVICE_INFO, 14, 175, 40]) + (123456).to_bytes(4, "little")
            r += b"Nov 20 2026".ljust(12, b"\0") + b"Elecrow ThinkNode M1".ljust(40, b"\0")
            r += b"v1.18.2".ljust(20, b"\0") + bytes([0, 0])
            return r
        r = bytes([RESP_SELF_INFO, 1, 22, 22]) + self.key + bytes(8) + bytes(4)
        r += (910525).to_bytes(4, "little") + (62500).to_bytes(4, "little") + bytes([7, 5])
        return r + self.name.encode()

    def reboot_and_reconnect(self, expect, timeout=40.0):
        self.name = self.saved_name


def selftest():
    # parsers
    assert parse_radio("910.525,62.5,7,5") == (910.525, 62.5, 7, 5)
    assert parse_radio("869.618,125,10,6") == (869.618, 125.0, 10, 6)
    assert parse_battery("3987 mV (85%)") == (3987, 85)
    for bad in ("", "> 910", "910.525,62.5,7"):
        try:
            parse_radio(bad)
        except AssertionError:
            continue
        raise SystemExit("parse_radio accepted %r" % bad)
    # frame codec
    payload = bytes(range(10))
    f = serial_cli.frame(payload)
    assert f == b"<\x0a\x00" + payload
    assert serial_cli.parse_frame(b"> 910.525,62.5,7,5\r\n>" + len(payload).to_bytes(2, "little") + payload + b"xx")[0] == payload
    assert serial_cli.parse_frame(b">\x05\x00ab")[0] is None
    assert serial_cli.clean_lines(b"get name\r\n  -> > alice\r\n", "get name") == ["alice"]
    assert serial_cli.clean_lines(b"get radio\r\n> 910.525,62.5,7,5\r\n", "get radio") == ["910.525,62.5,7,5"]

    quiet = open(os.devnull, "w")
    base = argparse.Namespace(internal_storage=False, gps_timeout=1, tx=True,
                              skip_reboot=False, soak=2)

    def statuses(dev, **over):
        a = argparse.Namespace(**{**vars(base), **over})
        return {r["name"]: r["status"] for r in run_checks(dev, a, out=quiet)}

    ok = statuses(FakeDevice())
    assert all(s == "PASS" for s in ok.values()), ok
    assert statuses(FakeDevice(internal=True), internal_storage=True)["storage"] == "PASS"

    # each failure mode must be caught by the matching check
    assert statuses(FakeDevice(qspi_id="000000"))["storage"] == "FAIL"
    assert statuses(FakeDevice(lfclk_src="rc"))["power"] == "FAIL"
    assert statuses(FakeDevice(gps_dead=True))["gps_uart"] == "FAIL"
    assert statuses(FakeDevice(battery="900 mV (0%)"))["battery"] == "FAIL"
    assert statuses(FakeDevice(board="Elecrow ThinkNode M1"))["identity"] == "FAIL"
    assert list(statuses(FakeDevice(board="Elecrow ThinkNode M1"))) == ["identity"], "must abort"
    skipped = statuses(FakeDevice(), tx=False, soak=0, skip_reboot=True)
    assert skipped["tx"] == skipped["soak"] == skipped["persistence"] == "SKIP", skipped

    # no hardware: exit code 3 and no traceback
    rc = main(["--port", "/nonexistent/ttyACM9"], out=quiet)
    assert rc == 3, rc
    print("self-test OK")
    return 0


# --- main -------------------------------------------------------------------

def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--port")
    p.add_argument("--internal-storage", action="store_true")
    p.add_argument("--gps-timeout", type=float, default=30.0)
    p.add_argument("--tx", action="store_true")
    p.add_argument("--skip-reboot", action="store_true")
    p.add_argument("--soak", type=float, default=0.0)
    p.add_argument("--flash", action="store_true")
    p.add_argument("--json")
    p.add_argument("--self-test", action="store_true")
    return p.parse_args(argv)


def flash(port, internal_storage):
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    build = [os.path.join(root, "tools", "build.sh"), "wio-tracker-l1-pro", "build", "--release"]
    if internal_storage:
        build += ["--features", "internal-storage"]
    subprocess.run(build, check=True)
    subprocess.run([os.path.join(root, "tools", "flash_serial.sh"), port or "",
                    "wio-tracker-l1-pro", "release"], check=True)


def main(argv=None, out=sys.stdout):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.self_test:
        return selftest()
    if args.flash:
        flash(args.port, args.internal_storage)
        time.sleep(5)

    deadline = time.time() + (60 if args.flash else 0)
    path = serial_cli.find_device(BOARD_NAME, args.port)
    while path is None and time.time() < deadline:
        time.sleep(2)
        path = serial_cli.find_device(BOARD_NAME, args.port)
    if path is None:
        print("no %s found (candidates: %s); pass --port or plug the board in"
              % (BOARD_NAME, ", ".join(serial_cli.candidate_ports()) or "none"), file=out)
        return 3

    print("using %s" % path, file=out)
    dev = serial_cli.SerialDevice(path)
    try:
        results = run_checks(dev, args, out=out)
    finally:
        dev.close()

    failed = [r for r in results if r["status"] == "FAIL"]
    print("%d passed, %d failed, %d skipped" % (
        sum(r["status"] == "PASS" for r in results), len(failed),
        sum(r["status"] == "SKIP" for r in results)), file=out)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"port": path, "results": results}, f, indent=2)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
