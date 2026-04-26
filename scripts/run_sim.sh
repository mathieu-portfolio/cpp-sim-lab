#!/usr/bin/env bash
set -e

if [ $# -lt 1 ]; then
    echo "Usage: ./scripts/run_sim.sh <simulation_name> [preset]"
    echo "Example: ./scripts/run_sim.sh particles_cpu debug"
    exit 1
fi

SIM_NAME=$1
PRESET=${2:-debug}

./scripts/build.sh "$PRESET"

BIN="build/$PRESET/simulations/$SIM_NAME/$PRESET/$SIM_NAME.exe"

if [ ! -f "$BIN" ]; then
    BIN="build/$PRESET/simulations/$SIM_NAME/Debug/$SIM_NAME.exe"
fi

if [ ! -f "$BIN" ]; then
    BIN="build/$PRESET/simulations/$SIM_NAME/Release/$SIM_NAME.exe"
fi

if [ ! -f "$BIN" ]; then
    echo "Could not find simulation binary for $SIM_NAME"
    exit 1
fi

echo "Running $SIM_NAME..."
"$BIN"
