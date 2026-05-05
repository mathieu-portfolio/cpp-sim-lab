#pragma once

#include "Vehicle.hpp"

namespace traffic_flow {

class Simulation;

struct VehicleIntent {
    float acceleration = 0.0f;
};

class DefaultVehicleBehavior {
public:
    VehicleIntent computeIntent(
        const Simulation& simulation,
        const Vehicle& vehicle,
        const Vehicle* leader,
        float leaderGap,
        bool mustStopBeforeCrossroad,
        float dt) const;
};

} // namespace traffic_flow
