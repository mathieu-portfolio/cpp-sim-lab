#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/plot_bench.sh <benchmark_name>"
    exit 1
fi

BENCH_NAME=$1
PRESET=${2:-release}

source "$ROOT_DIR/scripts/benchmark_registry.sh"

ENTRY="$(benchmark_registry_entry_by_id "$PRESET" "$BENCH_NAME")"
if [[ -z "$ENTRY" ]]; then
    echo "Error: benchmark not registered in CMake: $BENCH_NAME"
    exit 1
fi

IFS='|' read -r _ _ BENCH_DIR <<< "$ENTRY"

CSV="$BENCH_DIR/results/results.csv"
PLOT="$BENCH_DIR/plot.py"

if [[ ! -f "$CSV" ]]; then
    echo "Error: results CSV not found: $CSV"
    exit 1
fi

if [[ ! -f "$PLOT" ]]; then
    echo "Warning: plot.py not found for $BENCH_NAME"
    exit 0
fi

echo "Plotting $BENCH_NAME..."
python "$PLOT" "$CSV"
