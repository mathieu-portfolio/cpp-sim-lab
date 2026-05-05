#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET="debug"
TARGET=""

usage() {
    echo "Usage: ./scripts/build.sh [preset] [target]"
    echo "Example: ./scripts/build.sh debug-ninja crowd"
}

configure_preset_exists() {
    local preset="$1"
    cmake --list-presets=configure 2>/dev/null | grep -Eq "^[[:space:]]*\"${preset}\""
}

build_preset_exists() {
    local preset="$1"
    cmake --list-presets=build 2>/dev/null | grep -Eq "^[[:space:]]*\"${preset}\""
}

for arg in "$@"; do
    case "$arg" in
    -*)
        candidate="${arg#-}"
        if configure_preset_exists "$candidate" || build_preset_exists "$candidate"; then
            PRESET="$candidate"
        else
            echo "Error: unknown argument '$arg'"
            usage
            exit 1
        fi
        ;;
    *)
        if configure_preset_exists "$arg" || build_preset_exists "$arg"; then
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

if ! configure_preset_exists "$PRESET" && ! build_preset_exists "$PRESET"; then
    echo "Error: unknown CMake preset '$PRESET'"
    echo "Available configure presets:"
    cmake --list-presets=configure || true
    echo "Available build presets:"
    cmake --list-presets=build || true
    exit 1
fi

if configure_preset_exists "$PRESET"; then
    cmake --preset "$PRESET"
fi

echo "Building: $PRESET"
if build_preset_exists "$PRESET"; then
    if [[ -n "$TARGET" ]]; then
        cmake --build --preset "$PRESET" --target "$TARGET"
    else
        cmake --build --preset "$PRESET"
    fi
else
    BUILD_DIR="build/$PRESET"
    if [[ -n "$TARGET" ]]; then
        cmake --build "$BUILD_DIR" --target "$TARGET"
    else
        cmake --build "$BUILD_DIR"
    fi
fi
