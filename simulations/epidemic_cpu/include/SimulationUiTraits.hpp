#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<epidemic_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Agents", &epidemic_cpu::SimulationStats::agentCount},
        StatField{"Susceptible", &epidemic_cpu::SimulationStats::susceptibleCount},
        StatField{"Infected", &epidemic_cpu::SimulationStats::infectedCount},
        StatField{"Recovered", &epidemic_cpu::SimulationStats::recoveredCount},
        StatField{"Neighbor checks", &epidemic_cpu::SimulationStats::neighborChecks},
        StatField{"Candidates", &epidemic_cpu::SimulationStats::neighborCandidates},
        StatField{"Grid cells", &epidemic_cpu::SimulationStats::occupiedGridCells},
        StatField{"Rt x100", &epidemic_cpu::SimulationStats::approxRt, 100.0f},
        StatField{"Infected %", &epidemic_cpu::SimulationStats::infectedFraction, 100.0f},
        StatField{"Peak %", &epidemic_cpu::SimulationStats::peakInfectedFraction, 100.0f},
        StatField{"Extinction s", &epidemic_cpu::SimulationStats::timeToExtinction}
    );
};

template <>
struct ConfigUiTraits<epidemic_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"max speed", &epidemic_cpu::SimulationConfig::maxSpeed, 1.0f, 10.0f, 80.0f},
        TunableField{"jitter", &epidemic_cpu::SimulationConfig::steeringJitter, 0.0f, 5.0f, 40.0f},
        TunableField{"infection radius", &epidemic_cpu::SimulationConfig::infectionRadius, 1.0f, 8.0f, 40.0f},
        TunableField{"infection rate", &epidemic_cpu::SimulationConfig::infectionRate, 0.01f, 0.1f, 2.0f},
        TunableField{"recovery time", &epidemic_cpu::SimulationConfig::recoveryTime, 0.5f, 1.0f, 30.0f},
        TunableField{"initial inf ratio", &epidemic_cpu::SimulationConfig::initialInfectedRatio, 0.0f, 0.01f, 0.5f},
        TunableField{"cell size", &epidemic_cpu::SimulationConfig::gridCellSize, 1.0f, 10.0f, 50.0f}
    );
};

} // namespace simfw::ui
