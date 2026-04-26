#!/usr/bin/env bash
set -e

PRESET=${1:-debug}

echo "Configuring: $PRESET"
cmake --preset "$PRESET"

echo "Building: $PRESET"
cmake --build --preset "$PRESET"
