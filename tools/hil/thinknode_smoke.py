#!/usr/bin/env python3
"""Hardware-in-the-loop smoke test for the Elecrow ThinkNode M1.

Talks to a connected board over its USB CDC port (text CLI + companion
frames) and verifies the things host tests cannot: clocks, regulators, QSPI
flash, GPS UART, battery ADC, config persistence across a real reboot and the
binary companion handshake.

    ./tools/build.sh thinknode-m1 hil [options]

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
2 bad usage, 3 no ThinkNode M1 found.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nrf52_smoke  # noqa: E402

SPEC = nrf52_smoke.BoardSpec(
    board_name="ThinkNode M1",
    board_slug="thinknode-m1",
    qspi_id="c22815",  # platforms/nrf52/src/qspi.rs (MX25R1635F)
    qspi_name="MX25R1635F",
    lfclk_cfg="xtal(50ppm)",
    wrong_board_sample="Wio Tracker L1 Pro",
)


def main(argv=None, out=sys.stdout):
    return nrf52_smoke.main(SPEC, argv=argv, out=out, description=__doc__.split("\n\n")[0])


if __name__ == "__main__":
    sys.exit(main())
