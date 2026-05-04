#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<heat_grid_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Min temp", &heat_grid_cpu::SimulationStats::minTemperature},
        StatField{"Max temp", &heat_grid_cpu::SimulationStats::maxTemperature},
        StatField{"Avg temp", &heat_grid_cpu::SimulationStats::avgTemperature}
    );
};

template <>
struct ConfigUiTraits<heat_grid_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"diffusion", &heat_grid_cpu::SimulationConfig::diffusion, 0.01f, 0.01f, 0.25f},
        TunableField{"ambient", &heat_grid_cpu::SimulationConfig::ambientTemperature, 0.01f, -1.0f, 1.0f},
        TunableField{"cell size", &heat_grid_cpu::SimulationConfig::cellSize, 0.5f, 3.0f, 12.0f}
    );
};

} // namespace simfw::ui
