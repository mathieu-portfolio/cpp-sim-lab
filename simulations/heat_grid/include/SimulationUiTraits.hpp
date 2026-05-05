#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<heat_grid::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Min temp", &heat_grid::SimulationStats::minTemperature},
        StatField{"Max temp", &heat_grid::SimulationStats::maxTemperature},
        StatField{"Avg temp", &heat_grid::SimulationStats::avgTemperature}
    );
};

template <>
struct ConfigUiTraits<heat_grid::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"diffusion", &heat_grid::SimulationConfig::diffusion, 0.01f, 0.01f, 0.25f},
        TunableField{"advection x", &heat_grid::SimulationConfig::advectionX, 0.05f, -2.0f, 2.0f},
        TunableField{"advection y", &heat_grid::SimulationConfig::advectionY, 0.05f, -2.0f, 2.0f},
        TunableField{"ambient", &heat_grid::SimulationConfig::ambientTemperature, 0.01f, -1.0f, 1.0f},
        TunableField{"brush radius", &heat_grid::SimulationConfig::brushRadius, 0.25f, 0.5f, 12.0f},
        TunableField{"cell size", &heat_grid::SimulationConfig::cellSize, 0.5f, 3.0f, 12.0f}
    );
};

} // namespace simfw::ui
