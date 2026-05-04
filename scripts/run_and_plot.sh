#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_and_plot.sh <benchmark_name> [preset] [--build]"
    echo "Example: ./scripts/run_and_plot.sh boids_spatial_grid_compare release --build"
    exit 1
fi

BENCH_NAME="$1"
PRESET="release"
BUILD_FIRST=false

for arg in "${@:2}"; do
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
        echo "Usage: ./scripts/run_and_plot.sh <benchmark_name> [preset] [--build]"
        exit 1
        ;;
    esac
done

RUN_BENCH_ARGS=("$BENCH_NAME" "$PRESET")
if [[ "$BUILD_FIRST" == true ]]; then
    RUN_BENCH_ARGS+=("--build")
fi

./scripts/run_bench.sh "${RUN_BENCH_ARGS[@]}"
./scripts/plot_bench.sh "$BENCH_NAME" "$PRESET"
