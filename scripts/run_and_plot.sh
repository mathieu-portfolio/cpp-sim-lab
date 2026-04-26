#!/usr/bin/env bash
set -e

if [ $# -lt 2 ]; then
    echo "Usage: ./scripts/run_and_plot.sh <benchmark_name> <preset>"
    exit 1
fi

./scripts/run_bench.sh $1 $2
./scripts/plot_bench.sh $1
