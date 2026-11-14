#!/usr/bin/env bash
set -euo pipefail

# Script to build and flash ferrix firmware serially via Nordic Serial DFU
# Usage: ./tools/flash_serial.sh [port] [board] [profile]
# Examples:
#   ./tools/flash_serial.sh
#   ./tools/flash_serial.sh /dev/cu.usbmodem1101
#   ./tools/flash_serial.sh /dev/cu.usbmodem1101 thinknode-m1 release

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

PORT="${1:-}"
BOARD="${2:-thinknode-m1}"
PROFILE="${3:-release}"
TARGET="thumbv7em-none-eabihf"

# Locate adafruit-nrfutil
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

# Locate llvm-objcopy from rust toolchain
SYSROOT="$(rustc --print sysroot)"
OBJCOPY="$(find "${SYSROOT}" -name llvm-objcopy | head -n 1)"
if [ -z "${OBJCOPY}" ]; then
    echo "ERROR: llvm-objcopy not found in rust sysroot. Please run: rustup component add llvm-tools-preview" >&2
    exit 1
fi

# Auto-detect serial port if not provided
if [ -z "${PORT}" ]; then
    # Search for macOS (/dev/cu.usbmodem*) or Linux (/dev/ttyACM*) ports
    PORTS=(/dev/cu.usbmodem* /dev/ttyACM*)
    VALID_PORTS=()
    for p in "${PORTS[@]}"; do
        if [ -e "${p}" ]; then
            VALID_PORTS+=("${p}")
        fi
    done

    # Filter for ThinkNode / MeshCore port if multiple ports detected
    MESH_PORTS=()
    for p in "${VALID_PORTS[@]}"; do
        if [[ "${p}" =~ Ferrix|MESH|1101 ]]; then
            MESH_PORTS+=("${p}")
        fi
    done

    if [ ${#MESH_PORTS[@]} -eq 1 ]; then
        PORT="${MESH_PORTS[0]}"
        echo "==> Auto-detected ThinkNode M1 port: ${PORT}"
    elif [ ${#VALID_PORTS[@]} -eq 1 ]; then
        PORT="${VALID_PORTS[0]}"
        echo "==> Auto-detected serial port: ${PORT}"
    elif [ ${#VALID_PORTS[@]} -gt 1 ]; then
        echo "Multiple serial devices detected:"
        for p in "${VALID_PORTS[@]}"; do
            echo "  - ${p}"
        done
        echo "Please specify the port to flash, e.g.:"
        echo "  $0 ${VALID_PORTS[0]}"
        exit 1
    else
        echo "ERROR: No USB serial device detected (/dev/cu.usbmodem* or /dev/ttyACM*)." >&2
        echo "Please connect your ThinkNode M1 via USB-C or specify the port explicitly, e.g.:" >&2
        echo "  $0 /dev/cu.usbmodemXXXX" >&2
        exit 1
    fi
fi

echo "=========================================================="
echo " Building & Packaging ferrix for Serial DFU"
echo " Board:   ${BOARD}"
echo " Profile: ${PROFILE}"
echo " Port:    ${PORT}"
echo "=========================================================="

cd "${ROOT_DIR}"

CARGO_FLAGS=("--target" "${TARGET}" "--no-default-features" "--features" "${BOARD}")
if [ "${PROFILE}" = "release" ]; then
    CARGO_FLAGS+=("--release")
fi

cargo build -p ferrix-thinknode-m1 "${CARGO_FLAGS[@]}"

ELF="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix"
HEX="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix-${BOARD}.hex"
ZIP="${ROOT_DIR}/target/${TARGET}/${PROFILE}/ferrix-${BOARD}-dfu.zip"

echo "==> Converting ELF to Intel HEX format..."
"${OBJCOPY}" -O ihex "${ELF}" "${HEX}"

echo "==> Generating Nordic DFU package (.zip)..."
# Target nRF52840 (dev-type 0x0052) with SoftDevice S140 6.1.1 (sd-req 0x00B6)
"${NRFUTIL}" dfu genpkg \
    --dev-type 0x0052 \
    --sd-req 0x00B6 \
    --application "${HEX}" \
    "${ZIP}"

# Determine Python interpreter powering adafruit-nrfutil (contains pySerial)
NRFUTIL_PYTHON=""
if [ -f "${NRFUTIL}" ]; then
    NRFUTIL_PYTHON="$(head -n 1 "${NRFUTIL}" | sed 's/^#!//')"
fi

echo "==> Checking device state..."
# If Ferrix/MeshCore/MeshCore companion firmware is running, trigger 1200-baud touch to enter bootloader
if ioreg -p IOUSB -l -w 0 2>/dev/null | grep -E -q "Ferrix|MeshCore|MeshCore"; then
    echo "==> Ferrix/MeshCore application active. Sending 1200-baud touch to enter bootloader..."
    PYTHON_CMD=""
    if [ -x "${NRFUTIL_PYTHON}" ]; then
        PYTHON_CMD="${NRFUTIL_PYTHON}"
    elif [ -x "${HOME}/.local/share/uv/tools/adafruit-nrfutil/bin/python" ]; then
        PYTHON_CMD="${HOME}/.local/share/uv/tools/adafruit-nrfutil/bin/python"
    fi

    if [ -n "${PYTHON_CMD}" ]; then
        "${PYTHON_CMD}" -c "
import serial
try:
    s = serial.Serial('${PORT}', 115200, timeout=0.2)
    s.write(b'dfu\r\n')
    s.flush()
    s.close()
except Exception:
    pass
try:
    s = serial.Serial('${PORT}', 1200, timeout=0.5)
    s.close()
except Exception:
    pass
" 2>/dev/null || true
    fi
    sleep 1.0
    IN_BOOTLOADER=false
    for i in $(seq 1 40); do
        if ioreg -p IOUSB -l -w 0 2>/dev/null | grep -q "ThinkNodeM1"; then
            IN_BOOTLOADER=true
            break
        fi
        sleep 0.2
    done

    if [ "${IN_BOOTLOADER}" != "true" ]; then
        echo "ERROR: Device failed to enter bootloader via 1200-baud touch." >&2
        echo "Please double-click the reset button on your device to enter bootloader manually, then re-run this script." >&2
        exit 1
    fi
    # Allow macOS kernel to complete USB CDC re-enumeration
    sleep 2.0
    for p in /dev/cu.usbmodem*; do
        if [ -e "${p}" ] && [[ "${p}" != *HA0F* ]]; then
            PORT="${p}"
            break
        fi
    done
    echo "==> Bootloader active on ${PORT}"
fi

echo "==> Flashing to ${PORT} over Serial DFU..."
FLASH_LOG=$(mktemp)
if ! "${NRFUTIL}" --verbose dfu serial \
    --package "${ZIP}" \
    -p "${PORT}" \
    -b 115200 2>&1 | tee "${FLASH_LOG}"; then
    rm -f "${FLASH_LOG}"
    echo "ERROR: Serial flashing failed!" >&2
    exit 1
fi

if grep -q "Failed to upgrade target" "${FLASH_LOG}"; then
    rm -f "${FLASH_LOG}"
    echo "ERROR: adafruit-nrfutil reported an error during flashing." >&2
    exit 1
fi
rm -f "${FLASH_LOG}"

echo ""
echo "=========================================================="
echo " FLASH SUCCESSFUL!"
echo " Device rebooting into ferrix..."
echo "=========================================================="
