#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

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
@@ -89,46 +92,54 @@ int main() {
        ClearBackground(BLACK);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
EOF

cat > "$TEST_DIR/smoke_test.cpp" << EOF
#include <gtest/gtest.h>

TEST(${SIM_NAME}, smoke) {
    SUCCEED();
}
EOF

cat > "$TEST_DIR/CMakeLists.txt" << EOF
add_executable(${SIM_NAME}_tests
    smoke_test.cpp
)

target_link_libraries(${SIM_NAME}_tests
    PRIVATE
        ${SIM_NAME}_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(${SIM_NAME}_tests)
EOF

echo "Created simulation: ${SIM_NAME}"
echo
echo "Next steps:"
echo "  1. Add: add_subdirectory(${SIM_DIR})"
echo "  2. Add: add_subdirectory(${TEST_DIR})"
echo "  3. Add: add_subdirectory(${BENCH_DIR})"