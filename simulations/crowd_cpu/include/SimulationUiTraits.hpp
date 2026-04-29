#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<crowd_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Agents", &crowd_cpu::SimulationStats::agentCount},
        StatField{"Reached", &crowd_cpu::SimulationStats::reachedGoalCount},
        StatField{"Obstacles", &crowd_cpu::SimulationStats::obstacleCount},
        StatField{"Neighbor checks", &crowd_cpu::SimulationStats::neighborChecks},
        StatField{"Obstacle checks", &crowd_cpu::SimulationStats::obstacleChecks},
        StatField{"Candidates", &crowd_cpu::SimulationStats::neighborCandidates},
        StatField{"Obstacle candidates", &crowd_cpu::SimulationStats::obstacleCandidates},
        StatField{"Grid cells", &crowd_cpu::SimulationStats::occupiedGridCells}
    );
};

template <>
struct ConfigUiTraits<crowd_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"max speed", &crowd_cpu::SimulationConfig::maxSpeed, 1.0f, 30.0f, 100.0f},
        TunableField{"max force", &crowd_cpu::SimulationConfig::maxForce, 1.0f, 30.0f, 100.0f},
        TunableField{"flow", &crowd_cpu::SimulationConfig::flowWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &crowd_cpu::SimulationConfig::separationWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"obstacle", &crowd_cpu::SimulationConfig::obstacleAvoidanceWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"cell size", &crowd_cpu::SimulationConfig::gridCellSize, 1.0f, 10.0f, 40.0f}
    );
};

} // namespace simfw::ui
