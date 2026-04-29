#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<sand_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Particles", &sand_cpu::SimulationStats::particleCount},
        StatField{"Active chunks", &sand_cpu::SimulationStats::activeChunks},
        StatField{"Moved", &sand_cpu::SimulationStats::movedCells}
    );
};

template <>
struct ConfigUiTraits<sand_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"brush radius", &sand_cpu::SimulationConfig::brushRadius, 1.0f, 1.0f, 20.0f},
        TunableField{"spawn amount", &sand_cpu::SimulationConfig::spawnAmount, 1.0f, 1.0f, 100.0f},
        TunableField{"cell size", &sand_cpu::SimulationConfig::cellSize, 0.5f, 1.0f, 8.0f}
    );
};

} // namespace simfw::ui
