#!/usr/bin/env python3
"""Poll the ThinkNode M1 USB CLI and log when it stops responding.

Usage: cli_monitor.py <seconds> [logfile]
"""
import os
import select
import sys
import termios
import time

PORT = "/dev/cu.usbmodem1101"


def open_port():
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
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


def ask(fd, cmd, timeout=0.8):
    try:
        os.write(fd, (cmd + "\r\n").encode())
    except Exception as e:
        return None, "write:%s" % e
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                chunk = os.read(fd, 512)
            except BlockingIOError:
                continue
            except Exception as e:
                return buf, "read:%s" % e
            if not chunk:
                break
            buf += chunk
    return buf, None


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
    logfile = sys.argv[2] if len(sys.argv) > 2 else "/tmp/m1-cli-monitor.log"
    log = open(logfile, "w", buffering=1)

    def emit(msg):
        log.write("%.3f %s\n" % (time.time(), msg))

    emit("monitor start port=%s duration=%s" % (PORT, duration))
    fd = None
    last_recv = None
    misses = 0
    start = time.time()
    n = 0
    while time.time() - start < duration:
        n += 1
        present = os.path.exists(PORT)
        emit("port_present=%s" % present)
        if not present:
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
                fd = None
            emit("PORT GONE")
            misses = 0
            time.sleep(1.0)
            continue
        if fd is None:
            try:
                fd = open_port()
                emit("port opened")
                # After a reopen (typically a reboot after a panic reset),
                # immediately ask for the persisted panic record.
                time.sleep(0.5)
                out, err = ask(fd, "get panic", timeout=1.5)
                text = (out or b"").decode("utf-8", "replace").strip()
                emit("PANIC_QUERY err=%s out=%r" % (err, text))
            except Exception as e:
                emit("open failed: %s" % e)
                time.sleep(1.0)
                continue
        cmd = "ver" if n % 5 else "stats-packets"
        out, err = ask(fd, cmd)
        text = (out or b"").decode("utf-8", "replace").strip().replace("\r", " | ").replace("\n", " | ")
        if err:
            emit("cmd=%s ERR=%s out=%r" % (cmd, err, text))
            misses += 1
            if misses >= 3:
                emit("3 errors: closing port for reopen")
                try:
                    os.close(fd)
                except Exception:
                    pass
                fd = None
                misses = 0
                time.sleep(1.0)
                continue
        elif "->" in text:
            emit("cmd=%s OK %s" % (cmd, text))
            misses = 0
        else:
            emit("cmd=%s NO_RESPONSE %r" % (cmd, text))
            misses += 1
            if misses >= 3:
                # Likely a crash/reboot: drop the handle so the next iteration
                # reopens the port and queries the panic record.
                emit("3 misses: closing port for reopen")
                try:
                    os.close(fd)
                except Exception:
                    pass
                fd = None
                misses = 0
                time.sleep(1.0)
                continue
        if cmd == "stats-packets" and "recv" in text:
            try:
                recv = int(text.split('"recv":')[1].split(",")[0])
                if last_recv is not None:
                    emit("RECV delta=%d total=%d" % (recv - last_recv, recv))
                last_recv = recv
            except Exception:
                pass
        time.sleep(1.0)
    emit("monitor stop")
    log.close()


if __name__ == "__main__":
    main()
