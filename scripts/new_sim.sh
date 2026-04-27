#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <simulation_name>"
    echo "Example: $0 boids_cpu"
    exit 1
fi

SIM_NAME="$1"

SIM_DIR="simulations/${SIM_NAME}"
TEST_DIR="tests/simulations/${SIM_NAME}"
BENCH_DIR="benchmarks/simulations/${SIM_NAME}"

if [[ -d "$SIM_DIR" ]]; then
    echo "Error: simulation already exists: $SIM_DIR"
    exit 1
fi

mkdir -p "$SIM_DIR/include"
mkdir -p "$SIM_DIR/src"
mkdir -p "$TEST_DIR"
mkdir -p "$BENCH_DIR"

cat > "$SIM_DIR/CMakeLists.txt" << EOF
add_library(${SIM_NAME}_core
    src/Simulation.cpp
)

target_include_directories(${SIM_NAME}_core
    PUBLIC
        \${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${SIM_NAME}_core
    PUBLIC
        sim_framework
)

target_compile_features(${SIM_NAME}_core
    PUBLIC
        cxx_std_20
)

add_executable(${SIM_NAME}
    main.cpp
)

target_link_libraries(${SIM_NAME}
    PRIVATE
        ${SIM_NAME}_core
        raylib
)
EOF

cat > "$SIM_DIR/include/Simulation.hpp" << EOF
#pragma once

class Simulation {
public:
    void update(float dt);
};
EOF

cat > "$SIM_DIR/src/Simulation.cpp" << EOF
#include "Simulation.hpp"

void Simulation::update(float dt) {
    (void)dt;
}
EOF

cat > "$SIM_DIR/main.cpp" << EOF
#include "Simulation.hpp"

#include <raylib.h>

int main() {
    InitWindow(800, 800, "${SIM_NAME}");
    SetTargetFPS(60);

    Simulation simulation;

    while (!WindowShouldClose()) {
        simulation.update(GetFrameTime());

        BeginDrawing();
        ClearBackground(BLACK);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
EOF

cat > "$SIM_DIR/README.md" << EOF
# ${SIM_NAME}

TODO: describe the simulation.

## Goals

- Correctness first
- Simple baseline implementation
- Measurements before optimization

## Controls

TODO
EOF

cat > "$TEST_DIR/CMakeLists.txt" << EOF
# TODO: add tests for ${SIM_NAME}
EOF

cat > "$BENCH_DIR/CMakeLists.txt" << EOF
# TODO: add benchmarks for ${SIM_NAME}
EOF

echo "Created simulation: ${SIM_NAME}"
echo
echo "Next steps:"
echo "  1. Add: add_subdirectory(${SIM_DIR})"
echo "  2. Add: add_subdirectory(${TEST_DIR})"
echo "  3. Add: add_subdirectory(${BENCH_DIR})"