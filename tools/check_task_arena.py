#!/usr/bin/env python3
"""Verify that all Embassy tasks in an ELF fit inside the static task arena.

Embassy allocates `#[embassy_executor::task]` pools from a fixed static bump
arena (`task-arena-size-*`) at runtime during boot. If the sum of task pool
sizes exceeds the arena, the firmware links cleanly and then panics at boot.

This script inspects DWARF debug info (`debug = 2`) in the compiled ELF to
extract `Arena<N>` and every `TaskPool<..., K>` struct size and alignment, and
exits non-zero if the worst-case arena consumption exceeds `N`.

Works on macOS (`dwarfdump` / `llvm-dwarfdump`) and Linux (`readelf` /
`llvm-dwarfdump`) using only the Python standard library.
"""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable

TAG_RE = re.compile(r"\bDW_TAG_([a-z0-9_]+)\b")
ARENA_NAME_RE = re.compile(r"^Arena<(\d+)>$")
TASK_POOL_PREFIX = "TaskPool<"

# dwarfdump / llvm-dwarfdump attribute formats:
#   DW_AT_name ("TaskPool<...>")
#   DW_AT_byte_size (0x24c8)
#   DW_AT_alignment (8)
DWARFDUMP_NAME_RE = re.compile(r'DW_AT_name\s+\("([^"]+)"\)')
DWARFDUMP_INT_RE = re.compile(r"DW_AT_(byte_size|alignment)\s+\((0x[0-9a-fA-F]+|\d+)\)")

# GNU readelf --debug-dump=info attribute formats:
#   <123> DW_AT_name : (indirect string, offset: 0x456): TaskPool<...>
#   <123> DW_AT_name : TaskPool<...>
#   <127> DW_AT_byte_size : 9416
#   <129> DW_AT_alignment : 8
READELF_NAME_RE = re.compile(r"DW_AT_name\s*:\s*(?:\([^)]*\):\s*)?(.+?)\s*$")
READELF_INT_RE = re.compile(r"DW_AT_(byte_size|alignment)\s*:\s*(0x[0-9a-fA-F]+|\d+)")


def _parse_int(token: str) -> int:
    return int(token, 16) if token.startswith(("0x", "0X")) else int(token, 10)


def parse_dwarf_lines(lines: Iterable[str]) -> tuple[dict[int, int], dict[str, tuple[int, int]]]:
    """Extract `Arena<N>` capacities/alignments and `TaskPool` sizes/alignments.

    Returns:
        arenas: mapping of `arena_capacity_bytes -> arena_alignment`
        pools:  mapping of `task_pool_type_name -> (byte_size, alignment)`
    """
    arenas: dict[int, int] = {}
    pools: dict[str, tuple[int, int]] = {}

    in_struct = False
    name: str | None = None
    byte_size: int | None = None
    alignment: int | None = None

    def flush() -> None:
        nonlocal in_struct, name, byte_size, alignment
        if in_struct and name is not None and byte_size is not None:
            align = alignment if alignment is not None else 8
            m = ARENA_NAME_RE.match(name)
            if m:
                cap = int(m.group(1))
                arenas[cap] = min(arenas.get(cap, align), align)
            elif name.startswith(TASK_POOL_PREFIX):
                prev = pools.get(name)
                if prev is None or byte_size > prev[0]:
                    pools[name] = (byte_size, align)
        in_struct = False
        name = None
        byte_size = None
        alignment = None

    for line in lines:
        tag_m = TAG_RE.search(line)
        if tag_m:
            flush()
            in_struct = tag_m.group(1) == "structure_type"
            continue

        if not in_struct:
            continue

        if "DW_AT_name" in line:
            m = DWARFDUMP_NAME_RE.search(line) or READELF_NAME_RE.search(line)
            if m:
                name = m.group(1).strip()
        elif "DW_AT_byte_size" in line or "DW_AT_alignment" in line:
            m = DWARFDUMP_INT_RE.search(line) or READELF_INT_RE.search(line)
            if m:
                kind, val_s = m.group(1), m.group(2)
                if kind == "byte_size":
                    byte_size = _parse_int(val_s)
                else:
                    alignment = _parse_int(val_s)

    flush()
    return arenas, pools


def worst_case_arena_usage(
    arena_align: int, pools: dict[str, tuple[int, int]]
) -> tuple[int, int]:
    """Return `(raw_sum, worst_case_with_alignment_padding)`.

    Embassy's `Arena::alloc` bumps an internal index and aligns each allocation
    up to `align_of::<TaskPool>()`. When `Arena` is less aligned than the most
    aligned `TaskPool` (e.g. 4 vs 8), the first allocation can lose up to
    `max_align - arena_align` bytes; any pool whose size is not a multiple of
    `max_align` could also introduce padding before a subsequent 8-byte-aligned
    pool depending on spawn order.
    """
    if not pools:
        return 0, 0
    raw_sum = sum(sz for sz, _ in pools.values())
    max_align = max(al for _, al in pools.values())
    initial_pad = max(0, max_align - arena_align)
    inter_pad = sum(
        (max_align - (sz % max_align)) % max_align for sz, _ in pools.values()
    )
    return raw_sum, raw_sum + initial_pad + inter_pad


def find_dwarf_tool() -> list[str]:
    """Pick an available DWARF dumper command prefix."""
    for candidate in (
        ("llvm-dwarfdump", ["--debug-info"]),
        ("dwarfdump", ["--debug-info"]),
        ("readelf", ["--debug-dump=info"]),
    ):
        exe = shutil.which(candidate[0])
        if exe:
            return [exe, *candidate[1]]

    # Also check rustup's llvm-tools sysroot (installed on CI).
    rustc = shutil.which("rustc")
    if rustc:
        sysroot = subprocess.check_output([rustc, "--print", "sysroot"], text=True).strip()
        for p in Path(sysroot).rglob("llvm-dwarfdump"):
            if p.is_file():
                return [str(p), "--debug-info"]

    raise RuntimeError(
        "No DWARF tool found (tried llvm-dwarfdump, dwarfdump, readelf). "
        "Install binutils or rustup component llvm-tools."
    )


def format_task_name(pool_name: str) -> str:
    """Shorten `TaskPool<crate::mod::__foo_task::{async_fn_env#0}, 1>` for display."""
    inner = pool_name[len(TASK_POOL_PREFIX) :].rstrip(">")
    inner = inner.replace("::{async_fn_env#0}", "")
    return inner


def check_elf(elf_path: Path, min_headroom: int = 0) -> int:
    if not elf_path.is_file():
        print(f"ERROR: ELF not found: {elf_path}", file=sys.stderr)
        return 2

    cmd = [*find_dwarf_tool(), str(elf_path)]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        errors="replace",
    )
    assert proc.stdout is not None
    arenas, pools = parse_dwarf_lines(proc.stdout)
    stderr = proc.stderr.read() if proc.stderr else ""
    rc = proc.wait()
    if rc != 0:
        print(f"ERROR: {' '.join(cmd)} exited with {rc}:\n{stderr}", file=sys.stderr)
        return 2

    if not arenas:
        print(
            f"ERROR: no Embassy `Arena<N>` found in {elf_path} "
            "(is debug info enabled with `debug = 2`?)",
            file=sys.stderr,
        )
        return 1
    if not pools:
        print(f"ERROR: no Embassy `TaskPool<...>` found in {elf_path}", file=sys.stderr)
        return 1

    arena_cap = max(arenas.keys())
    arena_align = arenas[arena_cap]
    raw_sum, worst_case = worst_case_arena_usage(arena_align, pools)
    headroom = arena_cap - worst_case

    print(f"Embassy task arena check: {elf_path}")
    for name, (sz, al) in sorted(pools.items(), key=lambda kv: kv[1][0], reverse=True):
        print(f"  {sz:6d} B (align {al})  {format_task_name(name)}")
    print(
        f"  total = {raw_sum} B, worst-case aligned = {worst_case} B / "
        f"{arena_cap} B arena (headroom = {headroom} B)"
    )

    if worst_case > arena_cap:
        print(
            f"ERROR: Embassy task arena OVERFLOW by {worst_case - arena_cap} bytes! "
            f"Shrink task futures or raise `task-arena-size-*` above {worst_case}.",
            file=sys.stderr,
        )
        return 1
    if headroom < min_headroom:
        print(
            f"ERROR: Embassy task arena headroom ({headroom} B) is below "
            f"minimum required ({min_headroom} B).",
            file=sys.stderr,
        )
        return 1
    return 0


def run_self_test() -> int:
    sample_dwarfdump = """
0x000238fa:       DW_TAG_structure_type
                    DW_AT_name\t("TaskPool<ferrix::____embassy_main_task::{async_fn_env#0}, 1>")
                    DW_AT_byte_size\t(0x24c8)
                    DW_AT_alignment\t(8)

0x00023ae0:       DW_TAG_structure_type
                    DW_AT_name\t("Arena<40960>")
                    DW_AT_byte_size\t(0xa004)
                    DW_AT_alignment\t(4)

0x00023b00:       DW_TAG_structure_type
                    DW_AT_name\t("TaskPool<ferrix_nrf52::ble::__ble_companion_task_task::{async_fn_env#0}, 1>")
                    DW_AT_byte_size\t(0x2498)
                    DW_AT_alignment\t(8)
"""
    arenas, pools = parse_dwarf_lines(sample_dwarfdump.splitlines())
    assert arenas == {40960: 4}, arenas
    assert pools == {
        "TaskPool<ferrix::____embassy_main_task::{async_fn_env#0}, 1>": (0x24C8, 8),
        "TaskPool<ferrix_nrf52::ble::__ble_companion_task_task::{async_fn_env#0}, 1>": (
            0x2498,
            8,
        ),
    }, pools
    raw, worst = worst_case_arena_usage(4, pools)
    assert raw == 0x24C8 + 0x2498
    assert worst == raw + 4

    sample_readelf = """
 <2><100>: Abbrev Number: 12 (DW_TAG_structure_type)
    <101>   DW_AT_name        : (indirect string, offset: 0x10): Arena<16384>
    <105>   DW_AT_byte_size   : 16388
    <107>   DW_AT_alignment   : 4
 <2><200>: Abbrev Number: 12 (DW_TAG_structure_type)
    <201>   DW_AT_name        : (indirect string, offset: 0x20): TaskPool<app::__big_task::{async_fn_env#0}, 1>
    <205>   DW_AT_byte_size   : 16384
    <209>   DW_AT_alignment   : 8
 <3><210>: Abbrev Number: 13 (DW_TAG_member)
    <211>   DW_AT_name        : (indirect string, offset: 0x30): inner
"""
    arenas2, pools2 = parse_dwarf_lines(sample_readelf.splitlines())
    assert arenas2 == {16384: 4}, arenas2
    assert pools2 == {"TaskPool<app::__big_task::{async_fn_env#0}, 1>": (16384, 8)}, pools2
    raw2, worst2 = worst_case_arena_usage(4, pools2)
    assert raw2 == 16384
    # 4-byte aligned arena + 8-byte aligned 16384-byte pool needs 16388 > 16384!
    assert worst2 == 16388 and worst2 > 16384

    print("check_task_arena.py self-test: ok")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", nargs="?", type=Path, help="Path to firmware ELF binary")
    parser.add_argument(
        "--min-headroom",
        type=int,
        default=0,
        help="Fail if remaining arena headroom is below this many bytes",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run built-in parser and overflow detection tests",
    )
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()
    if args.elf is None:
        parser.error("ELF path is required unless --self-test is passed")
    return check_elf(args.elf, min_headroom=args.min_headroom)


if __name__ == "__main__":
    raise SystemExit(main())
