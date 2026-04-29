#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_and_plot.sh <benchmark_name> [preset]"
    echo "Example: ./scripts/run_and_plot.sh boids_spatial_grid_compare release"
    exit 1
fi

BENCH_NAME=$1
PRESET=${2:-release}

./scripts/run_bench.sh "$BENCH_NAME" "$PRESET"
./scripts/plot_bench.sh "$BENCH_NAME"