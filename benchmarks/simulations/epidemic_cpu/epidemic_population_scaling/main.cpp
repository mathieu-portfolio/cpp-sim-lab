#include <AdaptiveBenchmark.hpp>
#include <ProgressBar.hpp>
#include <iostream>
#include <vector>

#include "Simulation.hpp"

using namespace epidemic_cpu;

int main() {
    constexpr bench::AdaptiveFrameBudget budget{1000.0, 30, 600};
    std::vector<std::size_t> counts{250,500,1000,2000,5000,10000};
    std::cout << "simulation,entity_count,frames,total_ms,avg_frame_ms,peak_infected_fraction,time_to_extinction\n";
    bench::ProgressBar progress(counts.size());
    for (auto count : counts) {
        SimulationConfig cfg; cfg.agentCount = count; cfg.entityCount = count;
        Simulation sim(cfg);
        const auto run = bench::runAdaptiveFrames(budget, [&](){ sim.update(1.0f/60.0f); });
        auto st = sim.getStats();
        std::cout << "epidemic_cpu," << count << "," << run.frames << "," << run.totalMs << "," << run.avgFrameMs << "," << st.peakInfectedFraction << "," << st.timeToExtinction << "\n";
        progress.advance();
    }
    progress.finish();
    return 0;
}
