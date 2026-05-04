#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<epidemic_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        makeStatField("Agents", &epidemic_cpu::SimulationStats::agentCount),
        makeStatField("Susceptible", &epidemic_cpu::SimulationStats::susceptibleCount),
        makeStatField("Infected", &epidemic_cpu::SimulationStats::infectedCount),
        makeStatField("Recovered", &epidemic_cpu::SimulationStats::recoveredCount),
        makeStatField("Neighbor checks", &epidemic_cpu::SimulationStats::neighborChecks),
        makeStatField("Candidates", &epidemic_cpu::SimulationStats::neighborCandidates),
        makeStatField("Grid cells", &epidemic_cpu::SimulationStats::occupiedGridCells),
        makeStatField("Rt x100", &epidemic_cpu::SimulationStats::approxRt),
        makeStatField("Infected %", &epidemic_cpu::SimulationStats::infectedFraction),
        makeStatField("Peak %", &epidemic_cpu::SimulationStats::peakInfectedFraction),
        makeStatField("Extinction s", &epidemic_cpu::SimulationStats::timeToExtinction)
    );
};

template <>
struct ConfigUiTraits<epidemic_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        makeTunableField("max speed", &epidemic_cpu::SimulationConfig::maxSpeed, 1.0f, 10.0f, 80.0f),
        makeTunableField("jitter", &epidemic_cpu::SimulationConfig::steeringJitter, 0.0f, 5.0f, 40.0f),
        makeTunableField("infection radius", &epidemic_cpu::SimulationConfig::infectionRadius, 1.0f, 8.0f, 40.0f),
        makeTunableField("infection rate", &epidemic_cpu::SimulationConfig::infectionRate, 0.01f, 0.1f, 2.0f),
        makeTunableField("recovery time", &epidemic_cpu::SimulationConfig::recoveryTime, 0.5f, 1.0f, 30.0f),
        makeTunableField("initial inf ratio", &epidemic_cpu::SimulationConfig::initialInfectedRatio, 0.0f, 0.01f, 0.5f),
        makeTunableField("cell size", &epidemic_cpu::SimulationConfig::gridCellSize, 1.0f, 10.0f, 50.0f)
    );
};

} // namespace simfw::ui
