#include "Simulation.hpp"

#include <chrono>
#include <iostream>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}
}

int main() {
    constexpr int WarmupFrames = 30;
    constexpr int MeasuredFrames = 300;
    constexpr float Dt = 1.0f / 60.0f;

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

    std::cout
        << "boid_count,"
        << "frames,"
        << "total_ms,"
        << "avg_frame_ms,"
        << "neighbor_checks\n";

    for (std::size_t boidCount : boidCounts) {
        SimulationConfig config;
        config.boidCount = boidCount;

        Simulation sim(config);

        for (int i = 0; i < WarmupFrames; ++i) {
            sim.update(Dt);
        }

        const auto start = Clock::now();

        for (int i = 0; i < MeasuredFrames; ++i) {
            sim.update(Dt);
        }

        const auto end = Clock::now();

        const SimulationStats stats = sim.getStats();
        const double totalMs = elapsedMs(start, end);
        const double avgFrameMs = totalMs / static_cast<double>(MeasuredFrames);

        std::cout
            << boidCount << ","
            << MeasuredFrames << ","
            << totalMs << ","
            << avgFrameMs << ","
            << stats.neighborChecks
            << "\n";
    }

    return 0;
}