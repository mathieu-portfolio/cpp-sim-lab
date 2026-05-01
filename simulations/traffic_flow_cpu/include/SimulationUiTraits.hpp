#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<traffic_flow_cpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Throughput", &traffic_flow_cpu::SimulationStats::throughputPerSecond},
        StatField{"Avg speed", &traffic_flow_cpu::SimulationStats::averageSpeed},
        StatField{"Avg queue", &traffic_flow_cpu::SimulationStats::averageQueueLength}
    );
};

template <>
struct ConfigUiTraits<traffic_flow_cpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"desired speed", &traffic_flow_cpu::SimulationConfig::desiredSpeed, 1.0f, 1.0f, 60.0f},
        TunableField{"max accel", &traffic_flow_cpu::SimulationConfig::maxAcceleration, 0.1f, 0.1f, 4.0f},
        TunableField{"braking", &traffic_flow_cpu::SimulationConfig::comfortableBraking, 0.1f, 0.1f, 5.0f},
        TunableField{"headway", &traffic_flow_cpu::SimulationConfig::desiredTimeHeadway, 0.2f, 0.1f, 4.0f}
    );
};

} // namespace simfw::ui
