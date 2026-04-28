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
constexpr std::uint32_t BaseSeed = 5050u;
constexpr float Dt = 1.0f / 60.0f;

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t obstacleChecks = 0;
    std::size_t obstacleCandidates = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t occupiedObstacleGridCells = 0;
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
        stats.obstacleCandidates,
        stats.occupiedGridCells,
        stats.occupiedObstacleGridCells
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
        5000,
        7500,
        10000
    };

    std::cout
        << "agent_count,"
        << "obstacles,"
        << "frames,"
        << "cell_size,"
        << "single_total_ms,"
        << "single_avg_frame_ms,"
        << "parallel_total_ms,"
        << "parallel_avg_frame_ms,"
        << "speedup,"
        << "single_neighbor_checks,"
        << "parallel_neighbor_checks,"
        << "single_neighbor_candidates,"
        << "parallel_neighbor_candidates,"
        << "single_obstacle_checks,"
        << "parallel_obstacle_checks,"
        << "single_obstacle_candidates,"
        << "parallel_obstacle_candidates,"
        << "grid_occupied_cells,"
        << "obstacle_grid_occupied_cells\n";

    for (std::size_t agentCount : agentCounts) {
        SimulationConfig baseConfig;
        baseConfig.agentCount = agentCount;
        baseConfig.entityCount = agentCount;
        baseConfig.useSpatialGrid = true;
        baseConfig.gridCellSize = baseConfig.separationRadius;

        SimulationConfig singleThreadConfig = baseConfig;
        singleThreadConfig.useParallelUpdate = false;

        SimulationConfig parallelConfig = baseConfig;
        parallelConfig.useParallelUpdate = true;

        const std::uint32_t seed = bench::seedFor(BaseSeed, agentCount);

        const BenchResult singleThread = runBenchmark(
            singleThreadConfig,
            ObstacleCount,
            WarmupFrames,
            MeasuredFrames,
            seed
        );

        const BenchResult parallel = runBenchmark(
            parallelConfig,
            ObstacleCount,
            WarmupFrames,
            MeasuredFrames,
            seed
        );

        const double speedup = singleThread.avgFrameMs / parallel.avgFrameMs;

        std::cout
            << agentCount << ","
            << ObstacleCount << ","
            << MeasuredFrames << ","
            << baseConfig.gridCellSize << ","
            << singleThread.totalMs << ","
            << singleThread.avgFrameMs << ","
            << parallel.totalMs << ","
            << parallel.avgFrameMs << ","
            << speedup << ","
            << singleThread.neighborChecks << ","
            << parallel.neighborChecks << ","
            << singleThread.neighborCandidates << ","
            << parallel.neighborCandidates << ","
            << singleThread.obstacleChecks << ","
            << parallel.obstacleChecks << ","
            << singleThread.obstacleCandidates << ","
            << parallel.obstacleCandidates << ","
            << parallel.occupiedGridCells << ","
            << parallel.occupiedObstacleGridCells
            << "\n";
    }

    return 0;
}
