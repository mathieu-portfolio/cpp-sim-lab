#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-debug}

if ! cmake --list-presets | rg -q "\"$PRESET\""; then
    echo "Error: unknown CMake preset '$PRESET'"
    exit 1
fi

echo "Configuring: $PRESET"
cmake --preset "$PRESET"

echo "Building: $PRESET"
cmake --build --preset "$PRESET"