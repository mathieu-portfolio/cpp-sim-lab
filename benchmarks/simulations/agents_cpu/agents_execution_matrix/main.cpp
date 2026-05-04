#include "Simulation.hpp"

#include <AdaptiveBenchmark.hpp>
#include <BenchTimer.hpp>
#include <ProgressBar.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

using namespace agents_cpu;

namespace {
constexpr std::uint32_t BaseSeed = 3030u;
constexpr float Dt = 1.0f / 60.0f;
constexpr std::size_t ObstacleCount = 64;
constexpr std::size_t MaxNaiveEntityCount = 2000;

struct ExecutionMode {
    std::string_view backend;
    std::string_view threading;
    bool useSpatialGrid = true;
    bool useParallelUpdate = true;
};

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t frames = 0;
    std::size_t workChecks = 0;
    std::size_t workCandidates = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t resultCount = 0;
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

void addObstacles(Simulation& sim, std::size_t obstacleCount) {
    const SimulationConfig& config = sim.getConfig();

    for (std::size_t i = 0; i < obstacleCount; ++i) {
        sim.addObstacle(obstaclePosition(i, obstacleCount, config));
    }
}

BenchResult runBenchmark(
    SimulationConfig config,
    const ExecutionMode& mode,
    int warmupFrames,
    const bench::AdaptiveFrameBudget& frameBudget,
    std::uint32_t seed
) {
    Random::seed(seed);

    config.execution.useSpatialGrid = mode.useSpatialGrid;
    config.execution.useParallelUpdate = mode.useParallelUpdate;
    config.gridCellSize = config.separationRadius;

    Simulation sim(config);
    addObstacles(sim, ObstacleCount);
    sim.setTarget(Vec2{config.width * 0.85f, config.height * 0.5f});

    for (int i = 0; i < warmupFrames; ++i) {
        sim.update(Dt);
    }

    const bench::AdaptiveRunResult run = bench::runAdaptiveFrames(frameBudget, [&]() {
        sim.update(Dt);
    });

    const SimulationStats stats = sim.getStats();

    return BenchResult{
        run.totalMs,
        run.avgFrameMs,
        run.frames,
        stats.neighborChecks + stats.obstacleChecks + stats.obstacleOverlapChecks,
        stats.neighborCandidates + stats.obstacleCandidates,
        stats.occupiedGridCells + stats.occupiedObstacleGridCells,
        stats.arrivedCount
    };
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr bench::AdaptiveFrameBudget FrameBudget{1000.0, 30, 600};
    constexpr double SlowFrameThresholdMs = 40.0;

    const std::vector<std::size_t> entityCounts{
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

    const std::vector<ExecutionMode> modes{
        {"naive", "single_thread", false, false},
        {"naive", "parallel", false, true},
        {"grid", "single_thread", true, false},
        {"grid", "parallel", true, true}
    };

    const std::size_t totalCases = entityCounts.size() * modes.size();
    bench::ProgressBar progress(totalCases);

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

    std::vector<bool> skipMode(modes.size(), false);

    for (std::size_t entityCount : entityCounts) {
        SimulationConfig baseConfig;
        baseConfig.agentCount = entityCount;
        baseConfig.entityCount = entityCount;

        const std::uint32_t seed = bench::seedFor(BaseSeed, entityCount);
        double baselineAvgFrameMs = 0.0;

        for (std::size_t modeIndex = 0; modeIndex < modes.size(); ++modeIndex) {
            const ExecutionMode& mode = modes[modeIndex];
            if (skipMode[modeIndex]) {
                continue;
            }
            if (!mode.useSpatialGrid && entityCount > MaxNaiveEntityCount) {
                continue;
            }

            const BenchResult result = runBenchmark(
                baseConfig,
                mode,
                WarmupFrames,
                FrameBudget,
                seed
            );

            if (baselineAvgFrameMs == 0.0) {
                baselineAvgFrameMs = result.avgFrameMs;
            }

            const double relative = baselineAvgFrameMs / result.avgFrameMs;
            if (bench::exceedsSlowThreshold({result.totalMs, result.avgFrameMs, result.frames}, SlowFrameThresholdMs)) {
                skipMode[modeIndex] = true;
            }

            progress.advance();
            std::cout
                << "agents_cpu,"
                << entityCount << ","
                << mode.backend << ","
                << mode.threading << ","
                << result.frames << ","
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

    progress.finish();

    return 0;
}