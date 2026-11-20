#!/usr/bin/env bash
#
# Board selector for ferrix.
#
#   ./tools/build.sh <board> [build|run|test] [cargo args...]
#
# Examples:
#   ./tools/build.sh thinknode-m1 build --release
#   ./tools/build.sh thinknode-m1 run
#   ./tools/build.sh thinknode-m1 test
#   ./tools/build.sh wio-tracker-l1-pro build --release
#
# The target is always passed explicitly because .cargo/config.toml sets a
# default embedded target, which must not leak into host tests or other boards.

set -euo pipefail

board="${1:-}"
action="${2:-build}"
[[ -n "$board" ]] || {
    echo "usage: tools/build.sh <board> [build|run|test] [cargo args...]" >&2
    echo "boards: thinknode-m1 wio-tracker-l1-pro" >&2
    exit 2
}
shift $(( $# >= 2 ? 2 : $# )) || true

case "$board" in
    thinknode-m1)
        pkg="ferrix-thinknode-m1-companion"
        target="thumbv7em-none-eabihf"
        ;;
    wio-tracker-l1-pro)
        pkg="ferrix-wio-tracker-l1-pro-companion"
        target="thumbv7em-none-eabihf"
        ;;
    *)
        echo "unknown board: $board" >&2
        echo "boards: thinknode-m1 wio-tracker-l1-pro" >&2
        exit 2
        ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "$action" == "test" ]]; then
    host="$(rustc -vV | sed -n 's/host: //p')"
    python3 "$script_dir/check_task_arena.py" --self-test
    exec cargo test -p meshcore-wire -p ferrix-hal -p ferrix-core -p ferrix-runtime -p ferrix-sx1262 -p ferrix-ssd1681 -p ferrix-sh1106 -p ferrix-l76k --target "$host" "$@"
fi

if [[ "$action" == "build" ]]; then
    cargo build -p "$pkg" --target "$target" "$@"
    profile="debug"
    prev=""
    for arg in "$@"; do
        if [[ "$prev" == "--profile" ]]; then
            profile="$arg"
            prev=""
        elif [[ "$arg" == "--release" || "$arg" == "-r" ]]; then
            profile="release"
        elif [[ "$arg" == --profile=* ]]; then
            profile="${arg#--profile=}"
        elif [[ "$arg" == "--profile" ]]; then
            prev="--profile"
        fi
    done
    elf="$script_dir/../target/$target/$profile/ferrix"
    if [[ -f "$elf" ]]; then
        python3 "$script_dir/check_task_arena.py" "$elf"
    fi
    exit 0
fi

exec cargo "$action" -p "$pkg" --target "$target" "$@"
