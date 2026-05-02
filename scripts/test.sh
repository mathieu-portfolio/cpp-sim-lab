#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET="debug"
BUILD_FIRST=false

usage() {
    echo "Usage: ./scripts/test.sh [preset] [--build]"
    echo "Example: ./scripts/test.sh debug-ninja --build"
}

preset_exists() {
    local preset="$1"
    cmake --list-presets=configure 2>/dev/null | grep -Eq "^[[:space:]]*\"${preset}\""
}

for arg in "$@"; do
    case "$arg" in
    --build)
        BUILD_FIRST=true
        ;;
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
        else
            echo "Error: unknown argument '$arg'"
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

if [[ "$BUILD_FIRST" == true ]]; then
    ./scripts/build.sh "$PRESET"
else
    echo "Skipping build (pass --build to compile tests first)."
fi

echo "Running tests: $PRESET"
if cmake --list-presets=test 2>/dev/null | grep -Eq "^[[:space:]]*\"${PRESET}\""; then
    ctest --preset "$PRESET"
else
    ctest --test-dir "build/$PRESET" --build-config "$PRESET" --output-on-failure
fi
