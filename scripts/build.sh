#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET="debug"
TARGET=""

usage() {
    echo "Usage: ./scripts/build.sh [preset] [target]"
    echo "Example: ./scripts/build.sh debug-ninja crowd_cpu"
}

preset_exists() {
    local preset="$1"
    cmake --list-presets=configure 2>/dev/null | grep -Eq "^[[:space:]]*\"${preset}\""
}

for arg in "$@"; do
    case "$arg" in
    -*)
        candidate="${arg#-}"
        if preset_exists "$candidate"; then
            PRESET="$candidate"
        else
            echo "Error: unknown argument '$arg'"
            usage
            exit 1
        fi
        ;;
    *)
        if preset_exists "$arg"; then
            PRESET="$arg"
        elif [[ -z "$TARGET" ]]; then
            TARGET="$arg"
        else
            echo "Error: unknown or duplicate argument '$arg'"
            usage
            exit 1
        fi
        ;;
    esac
done

if ! preset_exists "$PRESET"; then
    echo "Error: unknown or invalid CMake configure preset '$PRESET'"
    echo "Available configure presets:"
    cmake --list-presets=configure || true
    exit 1
fi

cmake --preset "$PRESET"

echo "Building: $PRESET"
if [[ -n "$TARGET" ]]; then
    cmake --build --preset "$PRESET" --target "$TARGET"
else
    cmake --build --preset "$PRESET"
fi
