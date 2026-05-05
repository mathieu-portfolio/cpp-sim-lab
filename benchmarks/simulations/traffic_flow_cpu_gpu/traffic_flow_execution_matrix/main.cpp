#include "Simulation.hpp"

#include <AdaptiveBenchmark.hpp>
#include <ProgressBar.hpp>

#include <cstddef>
#include <iostream>
#include <string_view>
#include <vector>

using namespace traffic_flow_cpu_gpu;

namespace {
constexpr float Dt = 1.0f / 60.0f;

struct ExecutionMode {
    std::string_view backend;
    std::string_view threading;
    bool useParallelUpdate = true;
};

struct BenchResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t frames = 0;
    float throughputPerSecond = 0.0f;
    float averageSpeed = 0.0f;
    float averageQueueLength = 0.0f;
};

BenchResult runBenchmark(SimulationConfig config, const ExecutionMode& mode, int warmupFrames, const bench::AdaptiveFrameBudget& frameBudget) {
    config.execution.useParallelUpdate = mode.useParallelUpdate;
    Simulation sim(config);

    for (int i = 0; i < warmupFrames; ++i) sim.update(Dt);

    const bench::AdaptiveRunResult run = bench::runAdaptiveFrames(frameBudget, [&]() { sim.update(Dt); });

    const SimulationStats stats = sim.getStats();
    return BenchResult{run.totalMs, run.avgFrameMs, run.frames, stats.throughputPerSecond, stats.averageSpeed, stats.averageQueueLength};
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr bench::AdaptiveFrameBudget FrameBudget{1000.0, 30, 900};
    constexpr double SlowFrameThresholdMs = 40.0;

    const std::vector<std::size_t> entityCounts{40, 70, 100, 150, 200, 300, 400};
    const std::vector<ExecutionMode> modes{{"default", "single_thread", false}, {"default", "parallel", true}};

    const std::size_t totalCases = entityCounts.size() * modes.size();
    bench::ProgressBar progress(totalCases);

    std::cout << "simulation,entity_count,backend,threading,frames,total_ms,avg_frame_ms,relative_to_single,throughput_per_second,average_speed,average_queue_length\n";

    std::vector<bool> skipMode(modes.size(), false);

    for (std::size_t entityCount : entityCounts) {
        SimulationConfig baseConfig;
        baseConfig.vehicleCount = entityCount;

        double baselineAvgFrameMs = 0.0;
        for (std::size_t modeIndex = 0; modeIndex < modes.size(); ++modeIndex) {
            const ExecutionMode& mode = modes[modeIndex];
            if (skipMode[modeIndex]) continue;

            const BenchResult result = runBenchmark(baseConfig, mode, WarmupFrames, FrameBudget);
            if (baselineAvgFrameMs == 0.0) baselineAvgFrameMs = result.avgFrameMs;
            const double relative = baselineAvgFrameMs / result.avgFrameMs;
            if (bench::exceedsSlowThreshold({result.totalMs, result.avgFrameMs, result.frames}, SlowFrameThresholdMs)) skipMode[modeIndex] = true;

            progress.advance();
            std::cout << "traffic_flow_cpu_gpu," << entityCount << "," << mode.backend << "," << mode.threading << "," << result.frames << "," << result.totalMs << "," << result.avgFrameMs << "," << relative << "," << result.throughputPerSecond << "," << result.averageSpeed << "," << result.averageQueueLength << "\n";
        }
    }

    progress.finish();

    return 0;
}
