#include "Simulation.hpp"

#include <BenchTimer.hpp>

#include <cstddef>
#include <iostream>
#include <string_view>
#include <vector>

using namespace traffic_flow_cpu;

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
    float throughputPerSecond = 0.0f;
    float averageSpeed = 0.0f;
    float averageQueueLength = 0.0f;
};

BenchResult runBenchmark(SimulationConfig config, const ExecutionMode& mode, int warmupFrames, int measuredFrames) {
    config.execution.useParallelUpdate = mode.useParallelUpdate;
    Simulation sim(config);

    for (int i = 0; i < warmupFrames; ++i) sim.update(Dt);

    const double totalMs = bench::measureMs([&]() {
        for (int i = 0; i < measuredFrames; ++i) sim.update(Dt);
    });

    const SimulationStats stats = sim.getStats();
    return BenchResult{totalMs, totalMs / static_cast<double>(measuredFrames), stats.throughputPerSecond, stats.averageSpeed, stats.averageQueueLength};
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 600;

    const std::vector<std::size_t> entityCounts{40, 70, 100, 150, 200, 300, 400};
    const std::vector<ExecutionMode> modes{{"default", "single_thread", false}, {"default", "parallel", true}};

    std::cout << "simulation,entity_count,backend,threading,frames,total_ms,avg_frame_ms,relative_to_single,throughput_per_second,average_speed,average_queue_length\n";

    for (std::size_t entityCount : entityCounts) {
        SimulationConfig baseConfig;
        baseConfig.vehicleCount = entityCount;

        double baselineAvgFrameMs = 0.0;
        for (const ExecutionMode& mode : modes) {
            const BenchResult result = runBenchmark(baseConfig, mode, WarmupFrames, MeasuredFrames);
            if (baselineAvgFrameMs == 0.0) baselineAvgFrameMs = result.avgFrameMs;
            const double relative = baselineAvgFrameMs / result.avgFrameMs;

            std::cout << "traffic_flow_cpu," << entityCount << "," << mode.backend << "," << mode.threading << "," << MeasuredFrames << "," << result.totalMs << "," << result.avgFrameMs << "," << relative << "," << result.throughputPerSecond << "," << result.averageSpeed << "," << result.averageQueueLength << "\n";
        }
    }

    return 0;
}
