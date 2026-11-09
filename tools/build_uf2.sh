#!/usr/bin/env bash
set -euo pipefail

# Script to build and package ThinkNode M1 firmware as a flashable UF2 file
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

TARGET="thumbv7em-none-eabihf"
PROFILE="${1:-release}"

echo "==> Building firmware profile: ${PROFILE} for ${TARGET}..."
cd "${ROOT_DIR}"

if [ "${PROFILE}" = "release" ]; then
    cargo build --release --target "${TARGET}"
    ELF="${ROOT_DIR}/target/${TARGET}/release/thinknode-m1-node"
else
    cargo build --target "${TARGET}"
    ELF="${ROOT_DIR}/target/${TARGET}/debug/thinknode-m1-node"
fi

SYSROOT="$(rustc --print sysroot)"
OBJCOPY="$(find "${SYSROOT}" -name llvm-objcopy | head -n 1)"

if [ -z "${OBJCOPY}" ]; then
    echo "ERROR: llvm-objcopy not found in rust sysroot. Please run: rustup component add llvm-tools-preview" >&2
    exit 1
fi

BIN="${ROOT_DIR}/target/${TARGET}/${PROFILE}/thinknode-m1-node.bin"
UF2="${ROOT_DIR}/target/${TARGET}/${PROFILE}/thinknode-m1-node.uf2"

echo "==> Converting ELF to raw binary with llvm-objcopy..."
"${OBJCOPY}" -O binary "${ELF}" "${BIN}"

echo "==> Converting binary to UF2 package..."
python3 "${SCRIPT_DIR}/uf2conv.py" "${BIN}" -o "${UF2}" -b 0x26000

echo ""
echo "=========================================================="
echo " BUILD SUCCESSFUL!"
echo " UF2 File: ${UF2}"
echo "=========================================================="
echo "To flash your ThinkNode M1:"
echo " 1. Double-click the reset button on your ThinkNode M1."
echo " 2. A USB drive named 'THINKNODE' or 'NRF52BOOT' will mount."
echo " 3. Drag and drop 'thinknode-m1-node.uf2' onto that drive."
echo "=========================================================="
