#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <> struct StatsUiTraits<crowd::SimulationStats> {
  static constexpr auto fields = std::make_tuple(
      makeStatField("Agents", &crowd::SimulationStats::agentCount),
      makeStatField("Reached", &crowd::SimulationStats::reachedGoalCount),
      makeStatField("Neighbor checks",
                    &crowd::SimulationStats::neighborChecks),
      makeStatField("Candidates",
                    &crowd::SimulationStats::neighborCandidates),
      makeStatField("Agent grid cells",
                    &crowd::SimulationStats::occupiedGridCells));
};

template <> struct ConfigUiTraits<crowd::SimulationConfig> {
  static constexpr auto fields = std::make_tuple(
      makeTunableField("max speed", &crowd::SimulationConfig::maxSpeed,
                       1.0f, 30.0f, 200.0f),
      makeTunableField("max force", &crowd::SimulationConfig::maxForce,
                       1.0f, 30.0f, 300.0f),
      makeTunableField("flow", &crowd::SimulationConfig::flowWeight, 0.0f,
                       0.5f, 2.0f),
      makeTunableField("separation",
                       &crowd::SimulationConfig::separationWeight, 0.0f,
                       0.5f, 2.0f),
      makeTunableField("separation radius",
                       &crowd::SimulationConfig::separationRadius, 1.0f,
                       5.0f, 80.0f),
      makeTunableField("goal radius", &crowd::SimulationConfig::goalRadius,
                       1.0f, 2.0f, 40.0f),
      makeTunableField("cell size", &crowd::SimulationConfig::gridCellSize,
                       1.0f, 10.0f, 60.0f));
};

} // namespace simfw::ui
