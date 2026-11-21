#!/usr/bin/env bash
set -euo pipefail

# Build and package ferrix firmware as a Nordic/Adafruit DFU .zip for BLE OTA
# or serial flashing (adafruit-nrfutil / nRF Device Firmware Update).
# Usage: ./tools/build_ota.sh [board] [profile]
# Examples:
#   ./tools/build_ota.sh thinknode-m1 release
#   ./tools/build_ota.sh wio-tracker-l1-pro release

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOARD="${1:-thinknode-m1}"
PROFILE="${2:-release}"
TARGET="thumbv7em-none-eabihf"

case "$BOARD" in
    thinknode-m1)
        pkg="ferrix-thinknode-m1-companion"
        sd_req="0x00B6" # S140 6.1.1
        ;;
    wio-tracker-l1-pro)
        pkg="ferrix-wio-tracker-l1-pro-companion"
        sd_req="0x0123" # S140 7.3.0
        ;;
    *)
        echo "unknown board: $BOARD" >&2
        echo "boards: thinknode-m1 wio-tracker-l1-pro" >&2
        exit 2
        ;;
esac

NRFUTIL=""
if command -v adafruit-nrfutil &>/dev/null; then
    NRFUTIL="$(command -v adafruit-nrfutil)"
elif [ -x "${HOME}/.local/bin/adafruit-nrfutil" ]; then
    NRFUTIL="${HOME}/.local/bin/adafruit-nrfutil"
else
    echo "ERROR: adafruit-nrfutil not found." >&2
    echo "Please install it with: uv tool install adafruit-nrfutil" >&2
    exit 1
fi

SYSROOT="$(rustc --print sysroot)"
OBJCOPY="$(find "${SYSROOT}" -name llvm-objcopy | head -n 1)"
if [ -z "${OBJCOPY}" ]; then
    echo "ERROR: llvm-objcopy not found in rust sysroot. Please run: rustup component add llvm-tools-preview" >&2
    exit 1
fi

echo "==> Building ferrix for board: ${BOARD} [${PROFILE}] (${TARGET})..."
cd "${ROOT_DIR}"

CARGO_FLAGS=("--target" "${TARGET}")
if [ "${PROFILE}" = "release" ]; then
    CARGO_FLAGS+=("--release")
fi

cargo build -p "${pkg}" "${CARGO_FLAGS[@]}"

ELF="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix"
HEX="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix-${BOARD}.hex"
ZIP="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix-${BOARD}-dfu.zip"

python3 "${SCRIPT_DIR}/check_task_arena.py" "${ELF}"

echo "==> Converting ELF to Intel HEX with llvm-objcopy..."
"${OBJCOPY}" -O ihex "${ELF}" "${HEX}"

echo "==> Generating Nordic DFU package (.zip)..."
# Target nRF52840 (dev-type 0x0052) with the board's SoftDevice (sd-req from the board case above)
"${NRFUTIL}" dfu genpkg \
    --dev-type 0x0052 \
    --sd-req "${sd_req}" \
    --application "${HEX}" \
    "${ZIP}"

echo ""
echo "=========================================================="
echo " OTA DFU PACKAGE READY!"
echo " Board:    ${BOARD}"
echo " Package:  ${ZIP}"
echo "=========================================================="
echo "To flash over BLE OTA (requires OTAFIX bootloader):"
echo " 1. Transfer '${ZIP}' to your phone."
echo " 2. Put the device in BLE OTA mode via:"
echo "    - System page (09/09) FK2 double-click ('BLE OTA DFU'), or"
echo "    - Serial/CLI command 'start ota', or"
echo "    - nRF DFU / nRF Connect legacy buttonless DFU."
echo " 3. Flash '${ZIP}' to 'TNM1_DFU' using nRF Device Firmware Update."
echo "=========================================================="
