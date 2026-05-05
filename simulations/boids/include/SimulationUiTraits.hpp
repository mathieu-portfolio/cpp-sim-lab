#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<boids::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Boids", &boids::SimulationStats::boidCount},
        StatField{"Neighbor checks", &boids::SimulationStats::neighborChecks},
        StatField{"Candidates", &boids::SimulationStats::neighborCandidates},
        StatField{"Grid cells", &boids::SimulationStats::occupiedGridCells}
    );
};

template <>
struct ConfigUiTraits<boids::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"alignment", &boids::SimulationConfig::alignmentWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"cohesion", &boids::SimulationConfig::cohesionWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &boids::SimulationConfig::separationWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"wander", &boids::SimulationConfig::wanderWeight, 0.0f, 0.25f, 1.0f},
        TunableField{"wander jitter", &boids::SimulationConfig::wanderJitter, 0.0f, 0.2f, 1.0f},
        TunableField{"perception", &boids::SimulationConfig::perceptionRadius, 1.0f, 30.0f, 100.0f},
        TunableField{"sep radius", &boids::SimulationConfig::separationRadius, 1.0f, 30.0f, 100.0f},
        TunableField{"cell size", &boids::SimulationConfig::gridCellSize, 1.0f, 30.0f, 100.0f}
    );
};

} // namespace simfw::ui
