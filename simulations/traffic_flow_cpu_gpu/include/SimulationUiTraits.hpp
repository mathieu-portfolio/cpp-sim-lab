#pragma once

#include "Simulation.hpp"

#include <ui/SimulationUiTraits.hpp>

namespace simfw::ui {

template <>
struct StatsUiTraits<traffic_flow_cpu_gpu::SimulationStats> {
    static constexpr auto fields = std::make_tuple(
        StatField{"Throughput", &traffic_flow_cpu_gpu::SimulationStats::throughputPerSecond},
        StatField{"Avg speed", &traffic_flow_cpu_gpu::SimulationStats::averageSpeed},
        StatField{"Avg queue", &traffic_flow_cpu_gpu::SimulationStats::averageQueueLength}
    );
};

template <>
struct ConfigUiTraits<traffic_flow_cpu_gpu::SimulationConfig> {
    static constexpr auto fields = std::make_tuple(
        TunableField{"desired speed", &traffic_flow_cpu_gpu::SimulationConfig::desiredSpeed, 1.0f, 1.0f, 60.0f},
        TunableField{"max accel", &traffic_flow_cpu_gpu::SimulationConfig::maxAcceleration, 0.1f, 0.1f, 4.0f},
        TunableField{"braking", &traffic_flow_cpu_gpu::SimulationConfig::comfortableBraking, 0.1f, 0.1f, 5.0f},
        TunableField{"headway", &traffic_flow_cpu_gpu::SimulationConfig::desiredTimeHeadway, 0.2f, 0.1f, 4.0f},
        TunableField{"yield lookahead", &traffic_flow_cpu_gpu::SimulationConfig::crossroadYieldLookahead, 2.0f, 15.0f, 120.0f},
        TunableField{"reservation lookahead", &traffic_flow_cpu_gpu::SimulationConfig::crossroadReservationLookahead, 2.0f, 15.0f, 140.0f},
        TunableField{"stop radius", &traffic_flow_cpu_gpu::SimulationConfig::crossroadStopRadius, 1.0f, 4.0f, 30.0f},
        TunableField{"clear delay", &traffic_flow_cpu_gpu::SimulationConfig::crossroadClearDelay, 0.05f, 0.1f, 2.0f},
        TunableField{"deadlock breaker", &traffic_flow_cpu_gpu::SimulationConfig::crossroadDeadlockBreakerWait, 0.1f, 0.25f, 8.0f},
        TunableField{"spawn crossroad clearance", &traffic_flow_cpu_gpu::SimulationConfig::spawnCrossroadClearance, 2.0f, 0.0f, 100.0f},
        TunableField{"spawn gap", &traffic_flow_cpu_gpu::SimulationConfig::spawnMinimumGap, 1.0f, 6.0f, 60.0f},
        TunableField{"physics gap", &traffic_flow_cpu_gpu::SimulationConfig::physicsMinimumGap, 0.5f, 0.0f, 20.0f},
        TunableField{"road smoothing", &traffic_flow_cpu_gpu::SimulationConfig::roadSmoothingIterations, 1.0f, 0.0f, 5.0f},
        TunableField{"road point min dist", &traffic_flow_cpu_gpu::SimulationConfig::roadSmoothingMinPointDistance, 1.0f, 2.0f, 30.0f}
    );
};

} // namespace simfw::ui
