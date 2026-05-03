#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

using namespace bubbles_cpu;

namespace {
constexpr std::uint32_t BaseSeed = 7070u;
constexpr float Dt = 1.0f / 60.0f;

struct ExecutionMode {
    std::string_view backend;
    std::string_view threading;
    bool useParallelUpdate = true;
};

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t bubbleCount = 0;
    std::size_t interactionChecks = 0;
    std::size_t collisionsResolved = 0;
};

BenchResult runBenchmark(SimulationConfig config, const ExecutionMode& mode, int warmupFrames, int measuredFrames, std::uint32_t seed) {
    Random::seed(seed);
    config.execution.useParallelUpdate = mode.useParallelUpdate;

    Simulation sim(config);
    for (std::size_t i = 0; i < config.entityCount; ++i) {
        sim.spawn({config.width * 0.5f, config.height * 0.5f});
    }

    for (int i = 0; i < warmupFrames; ++i) {
        sim.update(Dt);
    }

    const double totalMs = bench::measureMs([&]() {
        for (int i = 0; i < measuredFrames; ++i) {
            sim.update(Dt);
        }
    });

    const SimulationStats stats = sim.getStats();
    return BenchResult{totalMs, totalMs / static_cast<double>(measuredFrames), stats.bubbleCount, stats.interactionChecks, stats.collisionsResolved};
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;

    const std::vector<std::size_t> entityCounts{100, 250, 500, 750, 1000, 1200};
    const std::vector<ExecutionMode> modes{{"default", "single_thread", false}, {"default", "parallel", true}};

    std::cout << "simulation,entity_count,backend,threading,frames,total_ms,avg_frame_ms,relative_to_single,bubble_count,interaction_checks,collisions_resolved\n";

    for (std::size_t entityCount : entityCounts) {
        SimulationConfig baseConfig;
        baseConfig.entityCount = entityCount;
        baseConfig.maxBubbleCount = entityCount;

        const std::uint32_t seed = bench::seedFor(BaseSeed, entityCount);
        double baselineAvgFrameMs = 0.0;

        for (const ExecutionMode& mode : modes) {
            const BenchResult result = runBenchmark(baseConfig, mode, WarmupFrames, MeasuredFrames, seed);
            if (baselineAvgFrameMs == 0.0) baselineAvgFrameMs = result.avgFrameMs;
            const double relative = baselineAvgFrameMs / result.avgFrameMs;

            std::cout << "bubbles_cpu," << entityCount << "," << mode.backend << "," << mode.threading << "," << MeasuredFrames << "," << result.totalMs << "," << result.avgFrameMs << "," << relative << "," << result.bubbleCount << "," << result.interactionChecks << "," << result.collisionsResolved << "\n";
        }
    }

    return 0;
}
