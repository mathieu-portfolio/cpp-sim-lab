#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source "$ROOT_DIR/scripts/find_bin.sh"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> [preset] [--build]"
    echo "Example: ./scripts/run_bench.sh boids_naive_neighbors release-ninja --build"
    exit 1
fi

BENCH_NAME="$1"
PRESET="release"
BUILD_FIRST=false

usage() {
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> [preset] [--build]"
    echo "Example: ./scripts/run_bench.sh boids_naive_neighbors release-ninja --build"
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

BUILD_DIR="build/$PRESET"
BENCH_SRC_DIR="benchmarks/simulations/*/$BENCH_NAME"
BENCH_SRC_DIR=$(echo $BENCH_SRC_DIR)
OUT_DIR="$BENCH_SRC_DIR/results"
OUT="$OUT_DIR/results.csv"
BENCH_EXE="${BENCH_NAME}_bench"
BENCHMARK_TIME_LIMIT_SEC="${BENCHMARK_TIME_LIMIT_SEC:-0}"

if ! [[ "$BENCHMARK_TIME_LIMIT_SEC" =~ ^[0-9]+$ ]]; then
    echo "Error: BENCHMARK_TIME_LIMIT_SEC must be a non-negative integer (seconds)."
    exit 1
fi

if [[ ! -d "$BENCH_SRC_DIR" ]]; then
    echo "Error: benchmark directory not found for $BENCH_NAME"
    exit 1
fi

if [[ "$BUILD_FIRST" == true ]]; then
    echo "Building benchmark target ($PRESET): $BENCH_EXE"
    ./scripts/build.sh "$PRESET" "$BENCH_EXE"
else
    echo "Skipping build (pass --build to compile $BENCH_EXE first)."
fi

if [[ ! -d "$BUILD_DIR" ]]; then
    echo "Error: build directory not found: $BUILD_DIR"
    echo "Run: cmake --preset $PRESET"
    exit 1
fi

BIN="$(
    find_bin "$BUILD_DIR" "$BENCH_EXE" \
        "$BUILD_DIR/benchmarks/simulations/$BENCH_NAME" \
        "$BUILD_DIR/benchmarks/simulations" \
        "$BUILD_DIR/bin"
)"

if [[ -z "$BIN" ]]; then
    echo "Error: could not find benchmark binary: $BENCH_EXE"
    echo "Searched under: $BUILD_DIR"
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "Running $BENCH_NAME..."
if (( BENCHMARK_TIME_LIMIT_SEC > 0 )); then
    if ! timeout "$BENCHMARK_TIME_LIMIT_SEC" "$BIN" > "$OUT"; then
        echo "Skipped $BENCH_NAME: exceeded BENCHMARK_TIME_LIMIT_SEC=${BENCHMARK_TIME_LIMIT_SEC}s"
        rm -f "$OUT"
        exit 0
    fi
else
    "$BIN" > "$OUT"
fi

echo "Saved to $OUT"
