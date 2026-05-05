#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<heat_grid_cpu_gpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Min temp", &heat_grid_cpu_gpu::SimulationStats::minTemperature},
        StatField{"Max temp", &heat_grid_cpu_gpu::SimulationStats::maxTemperature},
        StatField{"Avg temp", &heat_grid_cpu_gpu::SimulationStats::avgTemperature}
    );
};

template <>
struct ConfigUiTraits<heat_grid_cpu_gpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"diffusion", &heat_grid_cpu_gpu::SimulationConfig::diffusion, 0.01f, 0.01f, 0.25f},
        TunableField{"advection x", &heat_grid_cpu_gpu::SimulationConfig::advectionX, 0.05f, -2.0f, 2.0f},
        TunableField{"advection y", &heat_grid_cpu_gpu::SimulationConfig::advectionY, 0.05f, -2.0f, 2.0f},
        TunableField{"ambient", &heat_grid_cpu_gpu::SimulationConfig::ambientTemperature, 0.01f, -1.0f, 1.0f},
        TunableField{"brush radius", &heat_grid_cpu_gpu::SimulationConfig::brushRadius, 0.25f, 0.5f, 12.0f},
        TunableField{"cell size", &heat_grid_cpu_gpu::SimulationConfig::cellSize, 0.5f, 3.0f, 12.0f}
    );
};

} // namespace simfw::ui
