#include "Simulation.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
};

double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

BenchResult runBenchmark(SimulationConfig config, int warmupFrames, int measuredFrames) {
    Simulation sim(config);

    constexpr float Dt = 1.0f / 60.0f;

    for (int i = 0; i < warmupFrames; ++i) {
        sim.update(Dt);
    }

    const auto start = Clock::now();

    for (int i = 0; i < measuredFrames; ++i) {
        sim.update(Dt);
    }

    const auto end = Clock::now();

    const SimulationStats stats = sim.getStats();
    const double totalMs = elapsedMs(start, end);

    return BenchResult{
        totalMs,
        totalMs / static_cast<double>(measuredFrames),
        stats.neighborChecks,
        stats.neighborCandidates,
        stats.occupiedGridCells
    };
}
}

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;

    const std::vector<std::size_t> boidCounts{
        100,
        250,
        500,
        750,
        1000,
        1500,
        2000,
        3000
    };

    std::cout
        << "boid_count,"
        << "frames,"
        << "cell_size,"
        << "naive_total_ms,"
        << "naive_avg_frame_ms,"
        << "grid_total_ms,"
        << "grid_avg_frame_ms,"
        << "speedup,"
        << "naive_neighbor_checks,"
        << "grid_neighbor_checks,"
        << "grid_neighbor_candidates,"
        << "grid_occupied_cells\n";

    for (std::size_t boidCount : boidCounts) {
        SimulationConfig baseConfig;
        baseConfig.boidCount = boidCount;
        baseConfig.gridCellSize = baseConfig.perceptionRadius;

        SimulationConfig naiveConfig = baseConfig;
        naiveConfig.useSpatialGrid = false;

        SimulationConfig gridConfig = baseConfig;
        gridConfig.useSpatialGrid = true;

        const BenchResult naive = runBenchmark(
            naiveConfig,
            WarmupFrames,
            MeasuredFrames
        );

        const BenchResult grid = runBenchmark(
            gridConfig,
            WarmupFrames,
            MeasuredFrames
        );

        const double speedup = naive.avgFrameMs / grid.avgFrameMs;

        std::cout
            << boidCount << ","
            << MeasuredFrames << ","
            << baseConfig.gridCellSize << ","
            << naive.totalMs << ","
            << naive.avgFrameMs << ","
            << grid.totalMs << ","
            << grid.avgFrameMs << ","
            << speedup << ","
            << naive.neighborChecks << ","
            << grid.neighborChecks << ","
            << grid.neighborCandidates << ","
            << grid.occupiedGridCells
            << "\n";
    }

    return 0;
}
