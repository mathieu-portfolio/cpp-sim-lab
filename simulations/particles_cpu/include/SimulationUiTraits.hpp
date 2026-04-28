#pragma once

#include "Simulation.hpp"
#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Particles", &SimulationStats::particleCount},
        StatField{"Checks", &SimulationStats::collisionChecks},
        StatField{"Resolved", &SimulationStats::collisionsResolved}
    );
};

template <>
struct ConfigUiTraits<SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"gravity", &SimulationConfig::gravity, 0.0f, 20.0f, 100.0f},
        TunableField{"damping", &SimulationConfig::damping, 0.9f, 0.01f, 0.05f},
        TunableField{"cell size", &SimulationConfig::gridCellSize, 1.0f, 10.0f, 40.0f}
    );
};

} // namespace simfw::ui
