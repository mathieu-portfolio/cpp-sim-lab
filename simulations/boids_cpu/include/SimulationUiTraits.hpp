#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<boids_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Boids", &boids_cpu::SimulationStats::boidCount},
        StatField{"Neighbor checks", &boids_cpu::SimulationStats::neighborChecks},
        StatField{"Candidates", &boids_cpu::SimulationStats::neighborCandidates},
        StatField{"Grid cells", &boids_cpu::SimulationStats::occupiedGridCells}
    );
};

template <>
struct ConfigUiTraits<boids_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"alignment", &boids_cpu::SimulationConfig::alignmentWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"cohesion", &boids_cpu::SimulationConfig::cohesionWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &boids_cpu::SimulationConfig::separationWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"perception", &boids_cpu::SimulationConfig::perceptionRadius, 1.0f, 30.0f, 100.0f},
        TunableField{"sep radius", &boids_cpu::SimulationConfig::separationRadius, 1.0f, 30.0f, 100.0f},
        TunableField{"cell size", &boids_cpu::SimulationConfig::gridCellSize, 1.0f, 30.0f, 100.0f}
    );
};

} // namespace simfw::ui
