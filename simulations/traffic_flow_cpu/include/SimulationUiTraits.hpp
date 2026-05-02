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
        TunableField{"headway", &traffic_flow_cpu::SimulationConfig::desiredTimeHeadway, 0.2f, 0.1f, 4.0f},
        TunableField{"yield lookahead", &traffic_flow_cpu::SimulationConfig::crossroadYieldLookahead, 2.0f, 15.0f, 120.0f},
        TunableField{"reservation lookahead", &traffic_flow_cpu::SimulationConfig::crossroadReservationLookahead, 2.0f, 15.0f, 140.0f},
        TunableField{"stop radius", &traffic_flow_cpu::SimulationConfig::crossroadStopRadius, 1.0f, 4.0f, 30.0f},
        TunableField{"clear delay", &traffic_flow_cpu::SimulationConfig::crossroadClearDelay, 0.05f, 0.1f, 2.0f},
        TunableField{"deadlock breaker", &traffic_flow_cpu::SimulationConfig::crossroadDeadlockBreakerWait, 0.1f, 0.25f, 8.0f},
        TunableField{"spawn crossroad clearance", &traffic_flow_cpu::SimulationConfig::spawnCrossroadClearance, 2.0f, 0.0f, 100.0f},
        TunableField{"spawn gap", &traffic_flow_cpu::SimulationConfig::spawnMinimumGap, 1.0f, 6.0f, 60.0f},
        TunableField{"physics gap", &traffic_flow_cpu::SimulationConfig::physicsMinimumGap, 0.5f, 0.0f, 20.0f},
        TunableField{"road smoothing", &traffic_flow_cpu::SimulationConfig::roadSmoothingIterations, 1.0f, 0.0f, 5.0f},
        TunableField{"road point min dist", &traffic_flow_cpu::SimulationConfig::roadSmoothingMinPointDistance, 1.0f, 2.0f, 30.0f}
    );
};

} // namespace simfw::ui
