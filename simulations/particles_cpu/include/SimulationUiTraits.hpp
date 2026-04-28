#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<particles_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Particles", &particles_cpu::SimulationStats::particleCount},
        StatField{"Checks", &particles_cpu::SimulationStats::collisionChecks},
        StatField{"Resolved", &particles_cpu::SimulationStats::collisionsResolved}
    );
};

template <>
struct ConfigUiTraits<particles_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"gravity", &particles_cpu::SimulationConfig::gravity, 0.0f, 20.0f, 100.0f},
        TunableField{"damping", &particles_cpu::SimulationConfig::damping, 0.9f, 0.01f, 0.05f},
        TunableField{"bounce", &particles_cpu::SimulationConfig::bounce, -1.0f, 0.05f, 0.2f},
        TunableField{"cell size", &particles_cpu::SimulationConfig::gridCellSize, 1.0f, 10.0f, 40.0f}
    );
};

} // namespace simfw::ui
