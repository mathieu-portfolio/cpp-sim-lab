#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_sim.sh <simulation_name> [preset]"
    echo "Example: ./scripts/run_sim.sh particles_cpu debug"
    exit 1
fi

SIM_NAME=$1
PRESET=${2:-debug}

./scripts/build.sh "$PRESET"

SIM_BUILD_DIR="build/$PRESET/simulations/$SIM_NAME"
if [[ ! -d "$SIM_BUILD_DIR" ]]; then
    echo "Error: simulation build directory not found: $SIM_BUILD_DIR"
    exit 1
fi

BIN=$(find "$SIM_BUILD_DIR" -type f \( -name "$SIM_NAME" -o -name "$SIM_NAME.exe" \) | head -n 1)

if [[ -z "$BIN" ]]; then
    echo "Error: could not find simulation binary for $SIM_NAME in $SIM_BUILD_DIR"
    exit 1
fi

echo "Running $SIM_NAME..."
"$BIN"