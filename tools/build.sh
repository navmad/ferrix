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
#
# The target is always passed explicitly because .cargo/config.toml sets a
# default embedded target, which must not leak into host tests or other boards.

set -euo pipefail

board="${1:-}"
action="${2:-build}"
[[ -n "$board" ]] || {
    echo "usage: tools/build.sh <board> [build|run|test] [cargo args...]" >&2
    echo "boards: thinknode-m1" >&2
    exit 2
}
shift $(( $# >= 2 ? 2 : $# )) || true

case "$board" in
    thinknode-m1)
        pkg="ferrix-thinknode-m1"
        target="thumbv7em-none-eabihf"
        ;;
    *)
        echo "unknown board: $board" >&2
        echo "boards: thinknode-m1" >&2
        exit 2
        ;;
esac

if [[ "$action" == "test" ]]; then
    host="$(rustc -vV | sed -n 's/host: //p')"
    exec cargo test -p meshcore-wire -p ferrix-hal -p ferrix-core -p ferrix-runtime -p ferrix-sx1262 -p ferrix-ssd1681 -p ferrix-l76k --target "$host" "$@"
fi

exec cargo "$action" -p "$pkg" --target "$target" "$@"
