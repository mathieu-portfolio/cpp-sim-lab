#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<agents_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Agents", &agents_cpu::SimulationStats::agentCount},
        StatField{"Arrived", &agents_cpu::SimulationStats::arrivedCount},
        StatField{"Obstacles", &agents_cpu::SimulationStats::obstacleCount},
        StatField{"Neighbor checks", &agents_cpu::SimulationStats::neighborChecks},
        StatField{"Obstacle checks", &agents_cpu::SimulationStats::obstacleChecks},
        StatField{"Candidates", &agents_cpu::SimulationStats::neighborCandidates},
        StatField{"Obstacle candidates", &agents_cpu::SimulationStats::obstacleCandidates},
        StatField{"Grid cells", &agents_cpu::SimulationStats::occupiedGridCells},
        StatField{"Obstacle grid cells", &agents_cpu::SimulationStats::occupiedObstacleGridCells}
    );
};

template <>
struct ConfigUiTraits<agents_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"max speed", &agents_cpu::SimulationConfig::maxSpeed, 1.0f, 30.0f, 100.0f},
        TunableField{"max force", &agents_cpu::SimulationConfig::maxForce, 1.0f, 30.0f, 100.0f},
        TunableField{"seek", &agents_cpu::SimulationConfig::seekWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &agents_cpu::SimulationConfig::separationWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"obstacle avoid", &agents_cpu::SimulationConfig::obstacleAvoidanceWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"arrival radius", &agents_cpu::SimulationConfig::arrivalRadius, 1.0f, 20.0f, 80.0f},
        TunableField{"target radius", &agents_cpu::SimulationConfig::targetRadius, 1.0f, 5.0f, 20.0f},
        TunableField{"separation radius", &agents_cpu::SimulationConfig::separationRadius, 1.0f, 10.0f, 40.0f},
        TunableField{"obstacle radius", &agents_cpu::SimulationConfig::obstacleRadius, 1.0f, 8.0f, 24.0f},
        TunableField{"avoid radius", &agents_cpu::SimulationConfig::obstacleAvoidanceRadius, 1.0f, 20.0f, 80.0f},
        TunableField{"cell size", &agents_cpu::SimulationConfig::gridCellSize, 1.0f, 10.0f, 40.0f}
    );
};

} // namespace simfw::ui
