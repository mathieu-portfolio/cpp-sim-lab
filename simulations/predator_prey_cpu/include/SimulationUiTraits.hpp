#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<predator_prey_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Prey", &predator_prey_cpu::SimulationStats::preyCount},
        StatField{"Predators", &predator_prey_cpu::SimulationStats::predatorCount},
        StatField{"Catches", &predator_prey_cpu::SimulationStats::catches},
        StatField{"Grid cells", &predator_prey_cpu::SimulationStats::occupiedGridCells}
    );
};

template <>
struct ConfigUiTraits<predator_prey_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"align", &predator_prey_cpu::SimulationConfig::alignmentWeight, 0.0f, 0.5f, 2.5f},
        TunableField{"cohesion", &predator_prey_cpu::SimulationConfig::cohesionWeight, 0.0f, 0.5f, 2.0f},
        TunableField{"separation", &predator_prey_cpu::SimulationConfig::separationWeight, 0.0f, 0.5f, 3.0f},
        TunableField{"flee", &predator_prey_cpu::SimulationConfig::fleeWeight, 0.0f, 0.5f, 3.0f},
        TunableField{"chase", &predator_prey_cpu::SimulationConfig::chaseWeight, 0.0f, 0.5f, 3.0f},
        TunableField{"prey perception", &predator_prey_cpu::SimulationConfig::preyPerceptionRadius, 1.0f, 30.0f, 120.0f},
        TunableField{"pred perception", &predator_prey_cpu::SimulationConfig::predatorPerceptionRadius, 20.0f, 30.0f, 200.0f}
    );
};

} // namespace simfw::ui
