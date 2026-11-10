#!/usr/bin/env bash
set -euo pipefail

# Script to build and package ferrix firmware as a flashable UF2 file
# Usage: ./tools/build_uf2.sh [board] [profile]
# Examples:
#   ./tools/build_uf2.sh thinknode-m1 release

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BOARD="${1:-thinknode-m1}"
PROFILE="${2:-release}"
TARGET="thumbv7em-none-eabihf"

echo "==> Building ferrix for board: ${BOARD} [${PROFILE}] (${TARGET})..."
cd "${ROOT_DIR}"

CARGO_FLAGS=("--target" "${TARGET}" "--no-default-features" "--features" "${BOARD}")
if [ "${PROFILE}" = "release" ]; then
    CARGO_FLAGS+=("--release")
fi

cargo build "${CARGO_FLAGS[@]}"

ELF="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix"

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
python3 "${SCRIPT_DIR}/uf2conv.py" "${BIN}" -o "${UF2}" -b 0x26000

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
