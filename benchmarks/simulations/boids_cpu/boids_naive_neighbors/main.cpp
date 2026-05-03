#include "Simulation.hpp"

#include <BenchTimer.hpp>
#include <ProgressBar.hpp>
#include <BenchmarkRandom.hpp>
#include <random/Random.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

using namespace boids_cpu;

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;
    constexpr float Dt = 1.0f / 60.0f;
    constexpr std::uint32_t BaseSeed = 1337u;

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

    const std::size_t totalCases = boidCounts.size();
    bench::ProgressBar progress(totalCases);

    std::cout
        << "boid_count,"
        << "frames,"
        << "total_ms,"
        << "avg_frame_ms,"
        << "neighbor_checks\n";

    for (std::size_t boidCount : boidCounts) {
        Random::seed(bench::seedFor(BaseSeed, boidCount));

        SimulationConfig config;
        config.boidCount = boidCount;
        config.entityCount = boidCount;
        config.execution.useSpatialGrid = false;
        config.execution.useParallelUpdate = false;

        Simulation sim(config);

        for (int i = 0; i < WarmupFrames; ++i) {
            sim.update(Dt);
        }

        const double totalMs = bench::measureMs([&]() {
            for (int i = 0; i < MeasuredFrames; ++i) {
                sim.update(Dt);
            }
        });

        const SimulationStats stats = sim.getStats();
        const double avgFrameMs = totalMs / static_cast<double>(MeasuredFrames);

        progress.advance();
        std::cout
            << boidCount << ","
            << MeasuredFrames << ","
            << totalMs << ","
            << avgFrameMs << ","
            << stats.neighborChecks
            << "\n";
    }

    progress.finish();

    return 0;
}
