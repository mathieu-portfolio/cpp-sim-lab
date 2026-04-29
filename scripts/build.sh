#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET="debug"
TARGET=""

for arg in "$@"; do
    case "$arg" in
    debug|release|relwithdebinfo|minsizerel)
        PRESET="$arg"
        ;;
    -debug|-release|-relwithdebinfo|-minsizerel)
        PRESET="${arg#-}"
        ;;
    *)
        if [[ -z "$TARGET" ]]; then
            TARGET="$arg"
        else
            echo "Error: unknown or duplicate argument '$arg'"
            echo "Usage: ./scripts/build.sh [preset] [target]"
            exit 1
        fi
        ;;
    esac
done

if ! cmake --preset "$PRESET" >/dev/null; then
    echo "Error: unknown or invalid CMake preset '$PRESET'"
    exit 1
fi

echo "Building: $PRESET"
if [[ -n "$TARGET" ]]; then
    cmake --build --preset "$PRESET" --target "$TARGET"
else
    cmake --build --preset "$PRESET"
fi
