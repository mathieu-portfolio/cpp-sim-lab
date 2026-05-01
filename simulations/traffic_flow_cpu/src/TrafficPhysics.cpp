#include "TrafficPhysics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>

namespace traffic_flow_cpu {
namespace {

float wrapDistance(float s, float length) {
    if (length <= 0.0f) return 0.0f;
    float wrapped = std::fmod(s, length);
    if (wrapped < 0.0f) wrapped += length;
    return wrapped;
}

float vehicleClearanceDistance(const Vehicle& a, const Vehicle& b, float configuredMinimum) {
    return std::max(configuredMinimum, (a.length + b.length) * 0.5f + configuredMinimum);
}

float travelCoordinate(const RoadSegment& road, const Vehicle& vehicle) {
    const int direction = road.lanes[static_cast<std::size_t>(vehicle.laneId)].direction;
    return direction >= 0 ? wrapDistance(vehicle.s, road.length)
                          : wrapDistance(road.length - vehicle.s, road.length);
}

float roadCoordinateFromTravel(const RoadSegment& road, int laneId, float q) {
    const int direction = road.lanes[static_cast<std::size_t>(laneId)].direction;
    const float wrappedQ = wrapDistance(q, road.length);
    return direction >= 0 ? wrappedQ : wrapDistance(road.length - wrappedQ, road.length);
}

bool isNearCrossroad(const Simulation& simulation, const Vehicle& vehicle) {
    const RoadNetwork& network = simulation.getRoadNetwork();
    if (vehicle.roadId >= network.roads.size()) return false;

    const Vec2 position = simulation.sampleLanePosition(vehicle.roadId, vehicle.laneId, vehicle.s);
    const float radius = simulation.getConfig().crossroadStopRadius + simulation.getConfig().laneWidth;
    const float radiusSq = radius * radius;

    for (const Crossroad& crossroad : network.crossroads) {
        if ((position - crossroad.position).lengthSquared() <= radiusSq) {
            return true;
        }
    }

    return false;
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

void TrafficPhysics::enforceLaneOrderSpacing(
    const Simulation& simulation,
    const std::vector<Vehicle>& current,
    std::vector<Vehicle>& proposed,
    float minimumCenterDistance,
    float dt) {
    const RoadNetwork& network = simulation.getRoadNetwork();

    for (std::size_t roadId = 0; roadId < network.roads.size(); ++roadId) {
        const RoadSegment& road = network.roads[roadId];
        if (road.length <= 1.0f || road.lanes.empty()) continue;

        for (std::size_t laneId = 0; laneId < road.lanes.size(); ++laneId) {
            std::vector<std::size_t> laneVehicles;
            for (std::size_t i = 0; i < proposed.size(); ++i) {
                if (proposed[i].roadId == roadId && proposed[i].laneId == static_cast<int>(laneId)) {
                    laneVehicles.push_back(i);
                }
            }

            if (laneVehicles.size() < 2) continue;

            std::sort(laneVehicles.begin(), laneVehicles.end(), [&](std::size_t lhs, std::size_t rhs) {
                return travelCoordinate(road, proposed[lhs]) < travelCoordinate(road, proposed[rhs]);
            });

            // Closed lanes have no permanent front vehicle. Pick the largest gap
            // as the free space, then clamp every following car behind the car in
            // front. Unlike the old pairwise rejection, this preserves flow by
            // moving a car to the nearest legal position instead of freezing it.
            std::size_t largestGapIndex = 0;
            float largestGap = -1.0f;
            for (std::size_t sortedIndex = 0; sortedIndex < laneVehicles.size(); ++sortedIndex) {
                const std::size_t nextIndex = (sortedIndex + 1) % laneVehicles.size();
                const float q = travelCoordinate(road, proposed[laneVehicles[sortedIndex]]);
                float nextQ = travelCoordinate(road, proposed[laneVehicles[nextIndex]]);
                if (nextIndex == 0) nextQ += road.length;
                const float gap = nextQ - q;
                if (gap > largestGap) {
                    largestGap = gap;
                    largestGapIndex = sortedIndex;
                }
            }

            const std::size_t anchorSortedIndex = (largestGapIndex + 1) % laneVehicles.size();
            std::size_t leaderSortedIndex = anchorSortedIndex;
            float leaderQ = travelCoordinate(road, proposed[laneVehicles[leaderSortedIndex]]);

            for (std::size_t step = 1; step < laneVehicles.size(); ++step) {
                const std::size_t followerSortedIndex = (anchorSortedIndex + laneVehicles.size() - step) % laneVehicles.size();
                const std::size_t leaderVehicleIndex = laneVehicles[leaderSortedIndex];
                const std::size_t followerVehicleIndex = laneVehicles[followerSortedIndex];

                float followerQ = travelCoordinate(road, proposed[followerVehicleIndex]);
                while (followerQ > leaderQ) followerQ -= road.length;

                const float requiredGap = vehicleClearanceDistance(
                    proposed[followerVehicleIndex],
                    proposed[leaderVehicleIndex],
                    minimumCenterDistance);
                const float maxFollowerQ = leaderQ - requiredGap;

                if (followerQ > maxFollowerQ) {
                    proposed[followerVehicleIndex].s = roadCoordinateFromTravel(
                        road,
                        static_cast<int>(laneId),
                        maxFollowerQ);
                    proposed[followerVehicleIndex].speed = std::min(
                        proposed[followerVehicleIndex].speed,
                        proposed[leaderVehicleIndex].speed);
                    proposed[followerVehicleIndex].acceleration =
                        (proposed[followerVehicleIndex].speed - current[followerVehicleIndex].speed) /
                        std::max(0.05f, dt);
                    followerQ = maxFollowerQ;
                }

                leaderSortedIndex = followerSortedIndex;
                leaderQ = followerQ;
            }
        }
    }
}

void TrafficPhysics::enforceCrossroadFootprints(
    const Simulation& simulation,
    const std::vector<Vehicle>& current,
    std::vector<Vehicle>& proposed,
    float minimumCenterDistance,
    float dt) {
    std::vector<std::size_t> order(proposed.size());
    std::iota(order.begin(), order.end(), std::size_t{0});

    std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs) {
        return proposed[lhs].speed > proposed[rhs].speed;
    });

    std::vector<Vehicle> accepted;
    accepted.reserve(proposed.size());

    for (std::size_t vehicleIndex : order) {
        const bool candidateNearCrossroad = isNearCrossroad(simulation, proposed[vehicleIndex]);
        bool blocked = false;

        if (candidateNearCrossroad) {
            const Vec2 candidatePosition = simulation.sampleLanePosition(
                proposed[vehicleIndex].roadId,
                proposed[vehicleIndex].laneId,
                proposed[vehicleIndex].s);

            for (const Vehicle& other : accepted) {
                if (other.roadId == proposed[vehicleIndex].roadId && other.laneId == proposed[vehicleIndex].laneId) {
                    continue; // same-lane spacing was solved by the 1D lane pass
                }

                if (!isNearCrossroad(simulation, other)) continue;

                const Vec2 otherPosition = simulation.sampleLanePosition(other.roadId, other.laneId, other.s);
                const float requiredDistance = vehicleClearanceDistance(
                    proposed[vehicleIndex],
                    other,
                    minimumCenterDistance);
                if ((candidatePosition - otherPosition).lengthSquared() < requiredDistance * requiredDistance) {
                    blocked = true;
                    break;
                }
            }
        }

        if (blocked) {
            proposed[vehicleIndex] = current[vehicleIndex];
            proposed[vehicleIndex].speed = 0.0f;
            proposed[vehicleIndex].acceleration = -current[vehicleIndex].speed / std::max(0.05f, dt);
        }

        accepted.push_back(proposed[vehicleIndex]);
    }
}

void TrafficPhysics::enforceNoOverlap(
    const Simulation& simulation,
    const std::vector<Vehicle>& current,
    std::vector<Vehicle>& proposed,
    float minimumCenterDistance,
    float dt) {
    if (current.size() != proposed.size()) return;

    enforceLaneOrderSpacing(simulation, current, proposed, minimumCenterDistance, dt);
    enforceCrossroadFootprints(simulation, current, proposed, minimumCenterDistance, dt);
}

} // namespace traffic_flow_cpu
