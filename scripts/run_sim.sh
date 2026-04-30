#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source "$ROOT_DIR/scripts/find_bin.sh"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_sim.sh <simulation_name> [preset] [--build]"
    echo "Example: ./scripts/run_sim.sh particles_cpu debug-ninja --build"
    exit 1
fi

SIM_NAME="$1"
PRESET="debug"
BUILD_FIRST=false

usage() {
    echo "Usage: ./scripts/run_sim.sh <simulation_name> [preset] [--build]"
    echo "Example: ./scripts/run_sim.sh particles_cpu debug-ninja --build"
}

preset_exists() {
    local preset="$1"
    cmake --list-presets=configure 2>/dev/null | grep -Eq "^[[:space:]]*\"${preset}\""
}

for arg in "${@:2}"; do
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
    ./scripts/build.sh "$PRESET" "$SIM_NAME"
else
    echo "Skipping build (pass --build to compile $SIM_NAME first)."
fi

BUILD_DIR="build/$PRESET"

if [[ ! -d "$BUILD_DIR" ]]; then
    echo "Error: build directory not found: $BUILD_DIR"
    echo "Run: cmake --preset $PRESET"
    exit 1
fi

BIN="$(
    find_bin "$BUILD_DIR" "$SIM_NAME" \
        "$BUILD_DIR/simulations/$SIM_NAME" \
        "$BUILD_DIR/simulations" \
        "$BUILD_DIR/bin"
)"

if [[ -z "$BIN" ]]; then
    echo "Error: could not find simulation binary: $SIM_NAME"
    echo "Searched under: $BUILD_DIR"
    exit 1
fi

echo "Running $SIM_NAME..."
"$BIN"
