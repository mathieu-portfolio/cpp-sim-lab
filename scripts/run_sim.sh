#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source "$ROOT_DIR/scripts/find_bin.sh"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_sim.sh <simulation_name> [preset]"
    echo "Example: ./scripts/run_sim.sh particles_cpu debug"
    exit 1
fi

SIM_NAME="$1"
PRESET="${2:-debug}"

./scripts/build.sh "$PRESET"

BUILD_DIR="build/$PRESET"

if [[ ! -d "$BUILD_DIR" ]]; then
    echo "Error: build directory not found: $BUILD_DIR"
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