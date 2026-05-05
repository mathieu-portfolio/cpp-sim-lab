#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<bubbles::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Bubbles", &bubbles::SimulationStats::bubbleCount},
        StatField{"Checks", &bubbles::SimulationStats::interactionChecks},
        StatField{"Collisions", &bubbles::SimulationStats::collisionsResolved},
        StatField{"Merged", &bubbles::SimulationStats::mergedCount},
        StatField{"Burst", &bubbles::SimulationStats::burstCount}
    );
};

template <>
struct ConfigUiTraits<bubbles::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"buoyancy", &bubbles::SimulationConfig::buoyancy, 0.0f, 20.0f, 700.0f},
        TunableField{"damping", &bubbles::SimulationConfig::damping, 0.9f, 0.01f, 0.999f},
        TunableField{"pressure", &bubbles::SimulationConfig::pressureStrength, 0.0f, 2.0f, 120.0f},
        TunableField{"tension", &bubbles::SimulationConfig::surfaceTension, 0.0f, 2.0f, 160.0f},
        TunableField{"collision", &bubbles::SimulationConfig::collisionStiffness, 0.0f, 2.0f, 120.0f},
        TunableField{"burst stress", &bubbles::SimulationConfig::burstStressThreshold, 0.1f, 0.1f, 8.0f},
        TunableField{"cell size", &bubbles::SimulationConfig::gridCellSize, 2.0f, 4.0f, 40.0f}
    );
};

} // namespace simfw::ui
