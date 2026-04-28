#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

using namespace particles_cpu;

namespace {
constexpr std::uint32_t BaseSeed = 1010u;
constexpr float Dt = 1.0f / 60.0f;

struct ExecutionMode {
    std::string_view backend;
    std::string_view threading;
    bool useSpatialGrid = true;
    bool useParallelUpdate = true;
};

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t workChecks = 0;
    std::size_t workCandidates = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t resultCount = 0;
};

Vec2 spawnPosition(std::size_t index, std::size_t count, const SimulationConfig& config) {
    const std::size_t columns = 64;
    const float margin = config.particleRadius * 4.0f;
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

void populateParticles(Simulation& sim, std::size_t particleCount) {
    auto& config = sim.getConfig();
    config.spawnCount = 1;

    for (std::size_t i = 0; i < particleCount; ++i) {
        sim.spawn(spawnPosition(i, particleCount, config));
    }
}

BenchResult runBenchmark(
    SimulationConfig config,
    const ExecutionMode& mode,
    int warmupFrames,
    int measuredFrames,
    std::uint32_t seed
) {
    Random::seed(seed);

    config.execution.useSpatialGrid = mode.useSpatialGrid;
    config.execution.useParallelUpdate = mode.useParallelUpdate;
    config.gridCellSize = config.particleRadius * 4.0f;
    config.cellSize = config.gridCellSize;

    Simulation sim(config);
    populateParticles(sim, config.maxParticleCount);

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
        stats.collisionChecks,
        0,
        mode.useSpatialGrid ? sim.getGrid().getCells().size() : 0,
        stats.collisionsResolved
    };
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;

    const std::vector<std::size_t> entityCounts{
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

    const std::vector<ExecutionMode> modes{
        {"naive", "single_thread", false, false},
        {"naive", "parallel", false, true},
        {"grid", "single_thread", true, false},
        {"grid", "parallel", true, true}
    };

    std::cout
        << "simulation,"
        << "entity_count,"
        << "backend,"
        << "threading,"
        << "frames,"
        << "total_ms,"
        << "avg_frame_ms,"
        << "relative_to_naive_single,"
        << "work_checks,"
        << "work_candidates,"
        << "occupied_grid_cells,"
        << "result_count\n";

    for (std::size_t entityCount : entityCounts) {
        SimulationConfig baseConfig;
        baseConfig.maxParticleCount = entityCount;
        baseConfig.entityCount = entityCount;

        const std::uint32_t seed = bench::seedFor(BaseSeed, entityCount);
        double baselineAvgFrameMs = 0.0;

        for (const ExecutionMode& mode : modes) {
            const BenchResult result = runBenchmark(
                baseConfig,
                mode,
                WarmupFrames,
                MeasuredFrames,
                seed
            );

            if (baselineAvgFrameMs == 0.0) {
                baselineAvgFrameMs = result.avgFrameMs;
            }

            const double relative = baselineAvgFrameMs / result.avgFrameMs;

            std::cout
                << "particles_cpu," 
                << entityCount << ","
                << mode.backend << ","
                << mode.threading << ","
                << MeasuredFrames << ","
                << result.totalMs << ","
                << result.avgFrameMs << ","
                << relative << ","
                << result.workChecks << ","
                << result.workCandidates << ","
                << result.occupiedGridCells << ","
                << result.resultCount
                << "\n";
        }
    }

    return 0;
}
