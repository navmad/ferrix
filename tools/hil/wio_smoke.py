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
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nrf52_smoke  # noqa: E402

SPEC = nrf52_smoke.BoardSpec(
    board_name="Wio Tracker L1 Pro",
    board_slug="wio-tracker-l1-pro",
    qspi_id="856015",  # platforms/nrf52/src/qspi.rs (P25Q16H)
    qspi_name="P25Q16H",
    lfclk_cfg="xtal(20ppm)",
    wrong_board_sample="Elecrow ThinkNode M1",
)


def main(argv=None, out=sys.stdout):
    return nrf52_smoke.main(SPEC, argv=argv, out=out, description=__doc__.split("\n\n")[0])


if __name__ == "__main__":
    sys.exit(main())
