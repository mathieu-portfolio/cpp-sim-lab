#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <ProgressBar.hpp>
#include <BenchmarkRandom.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string_view>
#include <vector>

using namespace sand_cpu;

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
    std::size_t particleCount = 0;
    std::size_t activeChunks = 0;
    std::size_t movedCells = 0;
};

BenchResult runBenchmark(SimulationConfig config, const ExecutionMode& mode, int warmupFrames, int measuredFrames) {
    config.execution.useParallelUpdate = mode.useParallelUpdate;
    config.maxParticleCount = config.gridWidth * config.gridHeight;
    config.entityCount = config.maxParticleCount;

    Simulation sim(config);
    sim.reset();

    for (int i = 0; i < warmupFrames; ++i) sim.update(Dt);

    const double totalMs = bench::measureMs([&]() {
        for (int i = 0; i < measuredFrames; ++i) sim.update(Dt);
    });

    const SimulationStats stats = sim.getStats();
    return BenchResult{totalMs, totalMs / static_cast<double>(measuredFrames), stats.particleCount, stats.activeChunks, stats.movedCells};
}
} // namespace

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;

    const std::vector<std::size_t> gridSizes{128, 192, 256, 320, 384};
    const std::vector<ExecutionMode> modes{{"default", "single_thread", false}, {"default", "parallel", true}};

    const std::size_t totalCases = gridSizes.size() * modes.size();
    bench::ProgressBar progress(totalCases);

    std::cout << "simulation,entity_count,backend,threading,frames,total_ms,avg_frame_ms,relative_to_single,particle_count,active_chunks,moved_cells\n";

    for (std::size_t gridSize : gridSizes) {
        SimulationConfig baseConfig;
        baseConfig.gridWidth = gridSize;
        baseConfig.gridHeight = gridSize;

        const std::size_t entityCount = gridSize * gridSize;
        double baselineAvgFrameMs = 0.0;

        for (const ExecutionMode& mode : modes) {
            const BenchResult result = runBenchmark(baseConfig, mode, WarmupFrames, MeasuredFrames);
            if (baselineAvgFrameMs == 0.0) baselineAvgFrameMs = result.avgFrameMs;
            const double relative = baselineAvgFrameMs / result.avgFrameMs;

            progress.advance();
            std::cout << "sand_cpu," << entityCount << "," << mode.backend << "," << mode.threading << "," << MeasuredFrames << "," << result.totalMs << "," << result.avgFrameMs << "," << relative << "," << result.particleCount << "," << result.activeChunks << "," << result.movedCells << "\n";
        }
    }

    progress.finish();

    return 0;
}
