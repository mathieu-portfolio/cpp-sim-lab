#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<particles::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Particles", &particles::SimulationStats::particleCount},
        StatField{"Checks", &particles::SimulationStats::collisionChecks},
        StatField{"Resolved", &particles::SimulationStats::collisionsResolved}
    );
};

template <>
struct ConfigUiTraits<particles::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"gravity", &particles::SimulationConfig::gravity, 0.0f, 20.0f, 100.0f},
        TunableField{"damping", &particles::SimulationConfig::damping, 0.9f, 0.01f, 0.05f},
        TunableField{"bounce", &particles::SimulationConfig::bounce, -1.0f, 0.05f, 0.2f},
        TunableField{"cell size", &particles::SimulationConfig::gridCellSize, 1.0f, 10.0f, 40.0f}
    );
};

} // namespace simfw::ui
