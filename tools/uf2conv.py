#!/usr/bin/env python3
"""
UF2 conversion utility for ThinkNode M1 (Nordic nRF52840).
Packs raw binary firmware into UF2 blocks compatible with the Adafruit nRF52 UF2 Bootloader.
Family ID: 0xADA52840 (Nordic nRF52840)
Default Base Address: 0x00026000
"""

import struct
import sys
import argparse

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157  # Randomly selected
UF2_MAGIC_END    = 0x0AB16F30  # Closes block
UF2_FLAG_FAMILYID = 0x00002000
NRF52840_FAMILY_ID = 0xADA52840
DEFAULT_BASE_ADDR = 0x00026000

def bin_to_uf2(input_bytes, base_addr=DEFAULT_BASE_ADDR, family_id=NRF52840_FAMILY_ID):
    out = bytearray()
    num_blocks = (len(input_bytes) + 255) // 256

    for block_no in range(num_blocks):
        ptr = block_no * 256
        chunk = input_bytes[ptr:ptr + 256]
        # Pad chunk to 256 bytes with 0xFF
        padded_chunk = chunk + b'\xFF' * (256 - len(chunk))
        target_addr = base_addr + ptr

        header = struct.pack(
            "<IIIIIIII",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            UF2_FLAG_FAMILYID,
            target_addr,
            256,            # payload size
            block_no,       # block number
            num_blocks,     # total blocks
            family_id       # family ID
        )

        block = bytearray(header)
        block.extend(padded_chunk)
        # Pad from 256 + 32 = 288 up to 512 - 4 = 508 with zeros
        block.extend(b'\x00' * (512 - 4 - len(block)))
        block.extend(struct.pack("<I", UF2_MAGIC_END))
        out.extend(block)

    return out

def main():
    parser = argparse.ArgumentParser(description="Convert binary to UF2 for ThinkNode M1")
    parser.add_argument("input", help="Input binary (.bin) file")
    parser.add_argument("-o", "--output", help="Output UF2 (.uf2) file", required=True)
    parser.add_argument("-b", "--base", help="Base address (default: 0x26000)", type=lambda x: int(x, 0), default=DEFAULT_BASE_ADDR)
    parser.add_argument("-f", "--family", help="Family ID (default: 0xADA52840)", type=lambda x: int(x, 0), default=NRF52840_FAMILY_ID)

    args = parser.parse_args()

    with open(args.input, "rb") as f:
        data = f.read()

    uf2_data = bin_to_uf2(data, base_addr=args.base, family_id=args.family)

    with open(args.output, "wb") as f:
        f.write(uf2_data)

    print(f"Successfully generated {args.output} ({len(uf2_data) // 512} blocks, base address 0x{args.base:08X})")

if __name__ == "__main__":
    main()

