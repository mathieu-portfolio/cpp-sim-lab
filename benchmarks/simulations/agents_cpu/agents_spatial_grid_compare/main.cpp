#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace agents_cpu;

namespace {
constexpr std::uint32_t BaseSeed = 4242u;
constexpr float Dt = 1.0f / 60.0f;

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t obstacleChecks = 0;
    std::size_t occupiedGridCells = 0;
};

Vec2 obstaclePosition(std::size_t index, std::size_t count, const SimulationConfig& config) {
    const std::size_t columns = 8;
    const float margin = 80.0f;
    const float usableWidth = config.width - margin * 2.0f;
    const float usableHeight = config.height - margin * 2.0f;

    const std::size_t row = index / columns;
    const std::size_t column = index % columns;
    const std::size_t rows = (count + columns - 1) / columns;

    const float xStep = columns > 1
        ? usableWidth / static_cast<float>(columns - 1)
        : 0.0f;

    const float yStep = rows > 1
        ? usableHeight / static_cast<float>(rows - 1)
        : 0.0f;

    return Vec2{
        margin + static_cast<float>(column) * xStep,
        margin + static_cast<float>(row) * yStep
    };
}

void addObstacles(Simulation& sim, std::size_t count) {
    const SimulationConfig& config = sim.getConfig();

    for (std::size_t i = 0; i < count; ++i) {
        sim.addObstacle(obstaclePosition(i, count, config));
    }
}

BenchResult runBenchmark(
    SimulationConfig config,
    std::size_t obstacleCount,
    int warmupFrames,
    int measuredFrames,
    std::uint32_t seed
) {
    Random::seed(seed);

    Simulation sim(config);
    addObstacles(sim, obstacleCount);
    sim.setTarget(Vec2{config.width * 0.85f, config.height * 0.5f});

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
        stats.obstacleChecks,
        stats.occupiedGridCells
    };
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;
    constexpr std::size_t ObstacleCount = 64;

    const std::vector<std::size_t> agentCounts{
        100,
        250,
        500,
        750,
        1000,
        1500,
        2000,
        3000,
        5000
    };

    std::cout
        << "agent_count,"
        << "obstacles,"
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
        << "naive_obstacle_checks,"
        << "grid_obstacle_checks,"
        << "grid_occupied_cells\n";

    for (std::size_t agentCount : agentCounts) {
        SimulationConfig baseConfig;
        baseConfig.agentCount = agentCount;
        baseConfig.entityCount = agentCount;
        baseConfig.execution.useParallelUpdate = false;
        baseConfig.gridCellSize = baseConfig.separationRadius;

        SimulationConfig naiveConfig = baseConfig;
        naiveConfig.execution.useSpatialGrid = false;

        SimulationConfig gridConfig = baseConfig;
        gridConfig.execution.useSpatialGrid = true;

        const std::uint32_t seed = bench::seedFor(BaseSeed, agentCount);

        const BenchResult naive = runBenchmark(
            naiveConfig,
            ObstacleCount,
            WarmupFrames,
            MeasuredFrames,
            seed
        );

        const BenchResult grid = runBenchmark(
            gridConfig,
            ObstacleCount,
            WarmupFrames,
            MeasuredFrames,
            seed
        );

        const double speedup = naive.avgFrameMs / grid.avgFrameMs;

        std::cout
            << agentCount << ","
            << ObstacleCount << ","
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
            << naive.obstacleChecks << ","
            << grid.obstacleChecks << ","
            << grid.occupiedGridCells
            << "\n";
    }

    return 0;
}
