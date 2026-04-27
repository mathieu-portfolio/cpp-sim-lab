#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {
constexpr std::uint32_t BaseSeed = 1337u;

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
};

BenchResult runBenchmark(
    SimulationConfig config,
    int warmupFrames,
    int measuredFrames,
    std::uint32_t seed
) {
    Random::seed(seed);

    Simulation sim(config);

    constexpr float Dt = 1.0f / 60.0f;

    for (int i = 0; i < warmupFrames; ++i) {
        sim.update(Dt);
    }

    const double totalMs = bench::measureMs([&]() {
        for (int i = 0; i < measuredFrames; ++i) {
            sim.update(Dt);
        }
    });

    const SimulationStats stats = sim.getStats();

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
        baseConfig.entityCount = boidCount;
        baseConfig.gridCellSize = baseConfig.perceptionRadius;

        SimulationConfig naiveConfig = baseConfig;
        naiveConfig.useSpatialGrid = false;

        SimulationConfig gridConfig = baseConfig;
        gridConfig.useSpatialGrid = true;

        const std::uint32_t seed = bench::seedFor(BaseSeed, boidCount);

        const BenchResult naive = runBenchmark(
            naiveConfig,
            WarmupFrames,
            MeasuredFrames,
            seed
        );

        const BenchResult grid = runBenchmark(
            gridConfig,
            WarmupFrames,
            MeasuredFrames,
            seed
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
