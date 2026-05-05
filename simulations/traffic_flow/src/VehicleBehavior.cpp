#include "VehicleBehavior.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <cmath>

namespace traffic_flow {

VehicleIntent DefaultVehicleBehavior::computeIntent(
    const Simulation& simulation,
    const Vehicle& vehicle,
    const Vehicle* leader,
    float leaderGap,
    bool mustStopBeforeCrossroad,
    float dt) const {
    const SimulationConfig& config = simulation.getConfig();

    VehicleIntent intent;
    if (mustStopBeforeCrossroad) {
        intent.acceleration = -std::max(config.comfortableBraking, vehicle.speed / std::max(0.05f, dt));
        return intent;
    }

    const float freeRoadTerm = std::pow(
        std::max(0.0f, vehicle.speed / std::max(1.0f, config.desiredSpeed)),
        config.accelerationExponent);

    float interactionTerm = 0.0f;
    if (leader != nullptr) {
        const float deltaV = vehicle.speed - leader->speed;
        const float sqrtTerm = std::max(
            0.1f,
            2.0f * std::sqrt(std::max(0.0f, config.maxAcceleration * config.comfortableBraking)));
        const float desiredGap = config.minimumGap +
            vehicle.speed * config.desiredTimeHeadway +
            std::max(0.0f, vehicle.speed * deltaV / sqrtTerm);
        const float ratio = desiredGap / std::max(0.5f, leaderGap);
        interactionTerm = ratio * ratio;
    }

    intent.acceleration = config.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
    return intent;
}

} // namespace traffic_flow
