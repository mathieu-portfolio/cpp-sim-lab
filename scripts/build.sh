#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-debug}
TARGET=${2:-}

if ! cmake --preset "$PRESET" >/dev/null; then
    echo "Error: unknown or invalid CMake preset '$PRESET'"
    exit 1
fi

echo "Building: $PRESET"
if [[ -n "$TARGET" ]]; then
    cmake --build --preset "$PRESET" --target "$TARGET"
else
    cmake --build --preset "$PRESET"
fi
