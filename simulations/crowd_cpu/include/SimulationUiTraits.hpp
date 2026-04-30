#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <> struct StatsUiTraits<crowd_cpu::SimulationStats> {
  static constexpr auto fields = std::make_tuple(
      makeStatField("Agents", &crowd_cpu::SimulationStats::agentCount),
      makeStatField("Reached", &crowd_cpu::SimulationStats::reachedGoalCount),
      makeStatField("Neighbor checks",
                    &crowd_cpu::SimulationStats::neighborChecks),
      makeStatField("Candidates",
                    &crowd_cpu::SimulationStats::neighborCandidates),
      makeStatField("Agent grid cells",
                    &crowd_cpu::SimulationStats::occupiedGridCells));
};

template <> struct ConfigUiTraits<crowd_cpu::SimulationConfig> {
  static constexpr auto fields = std::make_tuple(
      makeTunableField("max speed", &crowd_cpu::SimulationConfig::maxSpeed,
                       1.0f, 30.0f, 200.0f),
      makeTunableField("max force", &crowd_cpu::SimulationConfig::maxForce,
                       1.0f, 30.0f, 300.0f),
      makeTunableField("flow", &crowd_cpu::SimulationConfig::flowWeight, 0.0f,
                       0.5f, 2.0f),
      makeTunableField("separation",
                       &crowd_cpu::SimulationConfig::separationWeight, 0.0f,
                       0.5f, 2.0f),
      makeTunableField("separation radius",
                       &crowd_cpu::SimulationConfig::separationRadius, 1.0f,
                       5.0f, 80.0f),
      makeTunableField("goal radius", &crowd_cpu::SimulationConfig::goalRadius,
                       1.0f, 2.0f, 40.0f),
      makeTunableField("cell size", &crowd_cpu::SimulationConfig::gridCellSize,
                       1.0f, 10.0f, 60.0f));
};

} // namespace simfw::ui
