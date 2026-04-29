#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET="debug"
BUILD_FIRST=false

for arg in "$@"; do
    case "$arg" in
    --build)
        BUILD_FIRST=true
        ;;
    debug|release|relwithdebinfo|minsizerel)
        PRESET="$arg"
        ;;
    -debug|-release|-relwithdebinfo|-minsizerel)
        PRESET="${arg#-}"
        ;;
    *)
        echo "Error: unknown argument '$arg'"
        echo "Usage: ./scripts/test.sh [preset] [--build]"
        exit 1
        ;;
    esac
done

if [[ "$BUILD_FIRST" == true ]]; then
    ./scripts/build.sh "$PRESET" test
else
    echo "Skipping build (pass --build to compile tests first)."
fi

echo "Running tests: $PRESET"
ctest --test-dir "build/$PRESET" --build-config "$PRESET" --output-on-failure
