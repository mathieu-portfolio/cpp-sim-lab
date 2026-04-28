#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Agents", &SimulationStats::agentCount},
        StatField{"Arrived", &SimulationStats::arrivedCount},
        StatField{"Neighbor checks", &SimulationStats::neighborChecks},
        StatField{"Candidates", &SimulationStats::neighborCandidates},
        StatField{"Grid cells", &SimulationStats::occupiedGridCells}
    );
};

template <>
struct ConfigUiTraits<SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"max speed", &SimulationConfig::maxSpeed, 1.0f, 30.0f, 100.0f},
        TunableField{"max force", &SimulationConfig::maxForce, 1.0f, 30.0f, 100.0f},
        TunableField{"seek", &SimulationConfig::seekWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &SimulationConfig::separationWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"arrival radius", &SimulationConfig::arrivalRadius, 1.0f, 20.0f, 80.0f},
        TunableField{"target radius", &SimulationConfig::targetRadius, 1.0f, 5.0f, 20.0f},
        TunableField{"separation radius", &SimulationConfig::separationRadius, 1.0f, 10.0f, 40.0f},
        TunableField{"cell size", &SimulationConfig::gridCellSize, 1.0f, 10.0f, 40.0f}
    );
};

} // namespace simfw::ui
