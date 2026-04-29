#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<bubbles_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Bubbles", &bubbles_cpu::SimulationStats::bubbleCount},
        StatField{"Checks", &bubbles_cpu::SimulationStats::interactionChecks},
        StatField{"Collisions", &bubbles_cpu::SimulationStats::collisionsResolved},
        StatField{"Merged", &bubbles_cpu::SimulationStats::mergedCount},
        StatField{"Burst", &bubbles_cpu::SimulationStats::burstCount}
    );
};

template <>
struct ConfigUiTraits<bubbles_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"buoyancy", &bubbles_cpu::SimulationConfig::buoyancy, 0.0f, 20.0f, 700.0f},
        TunableField{"damping", &bubbles_cpu::SimulationConfig::damping, 0.9f, 0.01f, 0.999f},
        TunableField{"pressure", &bubbles_cpu::SimulationConfig::pressureStrength, 0.0f, 2.0f, 120.0f},
        TunableField{"tension", &bubbles_cpu::SimulationConfig::surfaceTension, 0.0f, 2.0f, 160.0f},
        TunableField{"collision", &bubbles_cpu::SimulationConfig::collisionStiffness, 0.0f, 2.0f, 120.0f},
        TunableField{"cell size", &bubbles_cpu::SimulationConfig::gridCellSize, 2.0f, 4.0f, 40.0f}
    );
};

} // namespace simfw::ui
