#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-debug}

if ! cmake --preset "$PRESET" >/dev/null; then
    echo "Error: unknown or invalid CMake preset '$PRESET'"
    exit 1
fi

echo "Building: $PRESET"
cmake --build --preset "$PRESET"
