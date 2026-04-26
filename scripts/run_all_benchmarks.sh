#!/usr/bin/env bash
set -e

PRESET=${1:-release}

echo "Running all benchmarks with preset: $PRESET"

for dir in benchmarks/*; do
    if [ -d "$dir" ]; then
        NAME=$(basename "$dir")

        if [ -f "$dir/plot.py" ]; then
            echo "----------------------------------"
            echo "Benchmark: $NAME"
            ./scripts/run_and_plot.sh "$NAME" "$PRESET"
        fi
    fi
done

echo "All benchmarks done."
