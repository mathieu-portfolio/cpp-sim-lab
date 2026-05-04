#!/usr/bin/env bash

benchmark_registry_path() {
    local preset="$1"
    echo "build/$preset/benchmarks/registry.txt"
}

benchmark_registry_require() {
    local preset="$1"
    local registry
    registry="$(benchmark_registry_path "$preset")"

    if [[ ! -f "$registry" ]]; then
        echo "Error: benchmark registry not found: $registry"
        echo "Run: cmake --preset $preset"
        exit 1
    fi

    echo "$registry"
}

benchmark_registry_entry_by_id() {
    local preset="$1"
    local bench_id="$2"
    local registry
    registry="$(benchmark_registry_require "$preset")"

    awk -F'|' -v id="$bench_id" '$1 == id { print; exit }' "$registry"
}
