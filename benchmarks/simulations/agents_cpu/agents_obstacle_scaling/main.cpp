#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <ProgressBar.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

using namespace agents_cpu;

namespace {
constexpr std::uint32_t BaseSeed = 9001u;
constexpr float Dt = 1.0f / 60.0f;

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t obstacleChecks = 0;
    std::size_t obstacleOverlapChecks = 0;
    std::size_t obstacleCandidates = 0;
    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
};

Vec2 obstaclePosition(std::size_t index, std::size_t count, const SimulationConfig& config) {
    const std::size_t columns = 16;
    const float margin = 48.0f;
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
    sim.setTarget(Vec2{config.width * 0.5f, config.height * 0.85f});

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
        stats.obstacleChecks,
        stats.obstacleOverlapChecks,
        stats.obstacleCandidates,
        stats.neighborChecks,
        stats.neighborCandidates,
        stats.occupiedGridCells
    };
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;
    constexpr std::size_t AgentCount = 1500;

    const std::vector<std::size_t> obstacleCounts{
        0,
        8,
        16,
        32,
        64,
        128,
        256,
        512
    };

    const std::size_t totalCases = obstacleCounts.size();
    bench::ProgressBar progress(totalCases);

    std::cout
        << "agent_count,"
        << "obstacles,"
        << "frames,"
        << "total_ms,"
        << "avg_frame_ms,"
        << "obstacle_checks,"
        << "obstacle_overlap_checks,"
        << "obstacle_candidates,"
        << "neighbor_checks,"
        << "neighbor_candidates,"
        << "occupied_grid_cells\n";

    for (std::size_t obstacleCount : obstacleCounts) {
        SimulationConfig config;
        config.agentCount = AgentCount;
        config.entityCount = AgentCount;
        config.execution.useSpatialGrid = true;
        config.gridCellSize = config.separationRadius;

        const std::uint32_t seed = bench::seedFor(BaseSeed, obstacleCount);

        const BenchResult result = runBenchmark(
            config,
            obstacleCount,
            WarmupFrames,
            MeasuredFrames,
            seed
        );

        std::cout
            << AgentCount << ","
            << obstacleCount << ","
            << MeasuredFrames << ","
            << result.totalMs << ","
            << result.avgFrameMs << ","
            << result.obstacleChecks << ","
            << result.obstacleOverlapChecks << ","
            << result.obstacleCandidates << ","
            << result.neighborChecks << ","
            << result.neighborCandidates << ","
            << result.occupiedGridCells
            << "\n";
    }

    progress.finish();

    return 0;
}
