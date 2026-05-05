#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<bubbles_cpu_gpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Bubbles", &bubbles_cpu_gpu::SimulationStats::bubbleCount},
        StatField{"Checks", &bubbles_cpu_gpu::SimulationStats::interactionChecks},
        StatField{"Collisions", &bubbles_cpu_gpu::SimulationStats::collisionsResolved},
        StatField{"Merged", &bubbles_cpu_gpu::SimulationStats::mergedCount},
        StatField{"Burst", &bubbles_cpu_gpu::SimulationStats::burstCount}
    );
};

template <>
struct ConfigUiTraits<bubbles_cpu_gpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"buoyancy", &bubbles_cpu_gpu::SimulationConfig::buoyancy, 0.0f, 20.0f, 700.0f},
        TunableField{"damping", &bubbles_cpu_gpu::SimulationConfig::damping, 0.9f, 0.01f, 0.999f},
        TunableField{"pressure", &bubbles_cpu_gpu::SimulationConfig::pressureStrength, 0.0f, 2.0f, 120.0f},
        TunableField{"tension", &bubbles_cpu_gpu::SimulationConfig::surfaceTension, 0.0f, 2.0f, 160.0f},
        TunableField{"collision", &bubbles_cpu_gpu::SimulationConfig::collisionStiffness, 0.0f, 2.0f, 120.0f},
        TunableField{"burst stress", &bubbles_cpu_gpu::SimulationConfig::burstStressThreshold, 0.1f, 0.1f, 8.0f},
        TunableField{"cell size", &bubbles_cpu_gpu::SimulationConfig::gridCellSize, 2.0f, 4.0f, 40.0f}
    );
};

} // namespace simfw::ui
