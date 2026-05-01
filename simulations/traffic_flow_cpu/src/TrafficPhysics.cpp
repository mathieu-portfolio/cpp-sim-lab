#include "TrafficPhysics.hpp"

#include <algorithm>
#include <cstddef>
#include <numeric>

namespace traffic_flow_cpu {
namespace {

float vehicleClearanceDistance(const Vehicle& a, const Vehicle& b, float configuredMinimum) {
    return std::max(configuredMinimum, (a.length + b.length) * 0.5f + configuredMinimum);
}

} // namespace

bool TrafficPhysics::overlapsAny(
    const Simulation& simulation,
    const Vehicle& candidate,
    const std::vector<Vehicle>& occupied,
    float minimumCenterDistance) {
    const Vec2 candidatePosition = simulation.sampleLanePosition(candidate.roadId, candidate.laneId, candidate.s);

    for (const Vehicle& other : occupied) {
        const Vec2 otherPosition = simulation.sampleLanePosition(other.roadId, other.laneId, other.s);
        const float requiredDistance = vehicleClearanceDistance(candidate, other, minimumCenterDistance);
        if ((candidatePosition - otherPosition).lengthSquared() < requiredDistance * requiredDistance) {
            return true;
        }
    }

    return false;
}

bool TrafficPhysics::canSpawnVehicleAt(
    const Simulation& simulation,
    const std::vector<Vehicle>& vehicles,
    const Vehicle& candidate,
    float minimumCenterDistance) {
    return !overlapsAny(simulation, candidate, vehicles, minimumCenterDistance);
}

void TrafficPhysics::enforceNoOverlap(
    const Simulation& simulation,
    const std::vector<Vehicle>& current,
    std::vector<Vehicle>& proposed,
    float minimumCenterDistance,
    float dt) {
    if (current.size() != proposed.size()) return;

    std::vector<std::size_t> order(current.size());
    std::iota(order.begin(), order.end(), std::size_t{0});

    // Faster vehicles reserve first. This makes a vehicle that has committed to
    // a crossroad less likely to be blocked by a slower vehicle behind it, while
    // the invariant still prevents every overlap.
    std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
        return proposed[lhs].speed > proposed[rhs].speed;
    });

    std::vector<Vehicle> accepted;
    accepted.reserve(proposed.size());
    std::vector<bool> resolved(proposed.size(), false);

    for (std::size_t vehicleIndex : order) {
        std::vector<Vehicle> occupied = accepted;
        occupied.reserve(proposed.size() - 1);

        // Vehicles that have not been accepted yet still occupy their old
        // position. This conservative reservation is what prevents two cars from
        // moving into the same crossroad cell during the same frame.
        for (std::size_t i = 0; i < current.size(); ++i) {
            if (i == vehicleIndex || resolved[i]) continue;
            occupied.push_back(current[i]);
        }

        if (overlapsAny(simulation, proposed[vehicleIndex], occupied, minimumCenterDistance)) {
            proposed[vehicleIndex] = current[vehicleIndex];
            proposed[vehicleIndex].speed = 0.0f;
            proposed[vehicleIndex].acceleration = -current[vehicleIndex].speed / std::max(0.05f, dt);
        }

        accepted.push_back(proposed[vehicleIndex]);
        resolved[vehicleIndex] = true;
    }
}

} // namespace traffic_flow_cpu
