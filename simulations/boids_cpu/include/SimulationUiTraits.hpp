#pragma once

#include "Simulation.hpp"
#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Boids", &SimulationStats::boidCount},
        StatField{"Neighbor checks", &SimulationStats::neighborChecks},
        StatField{"Candidates", &SimulationStats::neighborCandidates},
        StatField{"Grid cells", &SimulationStats::occupiedGridCells}
    );
};

template <>
struct ConfigUiTraits<SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"alignment", &SimulationConfig::alignmentWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"cohesion", &SimulationConfig::cohesionWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &SimulationConfig::separationWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"perception", &SimulationConfig::perceptionRadius, 1.0f, 30.0f, 100.0f},
        TunableField{"sep radius", &SimulationConfig::separationRadius, 1.0f, 30.0f, 100.0f},
        TunableField{"cell size", &SimulationConfig::gridCellSize, 1.0f, 30.0f, 100.0f}
    );
};

} // namespace simfw::ui
