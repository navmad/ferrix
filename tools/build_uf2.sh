#!/usr/bin/env bash
set -euo pipefail

# Script to build and package ferrix firmware as a flashable UF2 file
# Usage: ./tools/build_uf2.sh [board] [profile]
# Examples:
#   ./tools/build_uf2.sh thinknode-m1 release
#   ./tools/build_uf2.sh wio-tracker-l1-pro release

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOARD="${1:-thinknode-m1}"
PROFILE="${2:-release}"
TARGET="thumbv7em-none-eabihf"

case "$BOARD" in
    thinknode-m1)
        pkg="ferrix-thinknode-m1-companion"
        # Application start: after the S140 6.1.1 SoftDevice.
        uf2_base="0x26000"
        ;;
    wio-tracker-l1-pro)
        pkg="ferrix-wio-tracker-l1-pro-companion"
        # Application start: after the S140 7.3.0 SoftDevice.
        uf2_base="0x27000"
        ;;
    *)
        echo "unknown board: $BOARD" >&2
        echo "boards: thinknode-m1 wio-tracker-l1-pro" >&2
        exit 2
        ;;
esac

echo "==> Building ferrix for board: ${BOARD} [${PROFILE}] (${TARGET})..."
cd "${ROOT_DIR}"

CARGO_FLAGS=("--target" "${TARGET}")
if [ "${PROFILE}" = "release" ]; then
    CARGO_FLAGS+=("--release")
fi

cargo build -p "${pkg}" "${CARGO_FLAGS[@]}"

ELF="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix"

python3 "${SCRIPT_DIR}/check_task_arena.py" "${ELF}"

SYSROOT="$(rustc --print sysroot)"
OBJCOPY="$(find "${SYSROOT}" -name llvm-objcopy | head -n 1)"

if [ -z "${OBJCOPY}" ]; then
    echo "ERROR: llvm-objcopy not found in rust sysroot. Please run: rustup component add llvm-tools-preview" >&2
    exit 1
fi

BIN="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix-${BOARD}.bin"
UF2="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix-${BOARD}.uf2"

echo "==> Converting ELF to raw binary with llvm-objcopy..."
"${OBJCOPY}" -O binary "${ELF}" "${BIN}"

echo "==> Converting binary to UF2 package..."
python3 "${SCRIPT_DIR}/uf2conv.py" "${BIN}" -o "${UF2}" -b "${uf2_base}"

echo ""
echo "=========================================================="
echo " BUILD SUCCESSFUL!"
echo " Board:    ${BOARD}"
echo " Firmware: ${UF2}"
echo "=========================================================="
echo "To flash your device:"
echo " 1. Double-click the reset button on your device to enter UF2 bootloader."
echo " 2. A USB mass-storage drive will mount automatically."
echo " 3. Drag and drop '${UF2}' onto that drive."
echo "=========================================================="
