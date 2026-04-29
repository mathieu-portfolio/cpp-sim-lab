#!/usr/bin/env bash

find_bin() {
    local build_dir="$1"
    local exe_name="$2"
    shift 2

    local preferred_dirs=("$@")
    local configs=("Debug" "Release" "RelWithDebInfo" "MinSizeRel")
    local exts=("" ".exe")

    local dir
    local config
    local ext
    local candidate

    for dir in "${preferred_dirs[@]}"; do
        for ext in "${exts[@]}"; do
            candidate="$dir/$exe_name$ext"
            if [[ -f "$candidate" ]]; then
                echo "$candidate"
                return 0
            fi
        done

        for config in "${configs[@]}"; do
            for ext in "${exts[@]}"; do
                candidate="$dir/$config/$exe_name$ext"
                if [[ -f "$candidate" ]]; then
                    echo "$candidate"
                    return 0
                fi
            done
        done
    done

    find "$build_dir" -maxdepth 6 -type f \
        \( -name "$exe_name" -o -name "$exe_name.exe" \) \
        -print -quit
}