#include "TrafficPhysics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>

namespace traffic_flow {
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
    if (vehicle.laneId < 0 || static_cast<std::size_t>(vehicle.laneId) >= road.lanes.size()) {
        return wrapDistance(vehicle.s, road.length);
    }
    const int direction = road.lanes[static_cast<std::size_t>(vehicle.laneId)].direction;
    return direction >= 0 ? wrapDistance(vehicle.s, road.length)
                          : wrapDistance(road.length - vehicle.s, road.length);
}

float roadCoordinateFromTravel(const RoadSegment& road, int laneId, float q) {
    if (laneId < 0 || static_cast<std::size_t>(laneId) >= road.lanes.size()) {
        return wrapDistance(q, road.length);
    }
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

void TrafficPhysics::enforceGlobalFootprintProjection(
    const Simulation& simulation,
    const std::vector<Vehicle>& current,
    std::vector<Vehicle>& proposed,
    float minimumCenterDistance,
    float dt) {
    const RoadNetwork& network = simulation.getRoadNetwork();
    if (proposed.size() < 2) return;

    auto positionOf = [&](const Vehicle& vehicle) {
        return simulation.sampleLanePosition(vehicle.roadId, vehicle.laneId, vehicle.s);
    };

    auto moveBackwardAlongLane = [&](Vehicle& vehicle, float amount) {
        if (vehicle.roadId >= network.roads.size()) return;
        const RoadSegment& road = network.roads[vehicle.roadId];
        if (road.length <= 0.0f || vehicle.laneId < 0 ||
            static_cast<std::size_t>(vehicle.laneId) >= road.lanes.size()) {
            return;
        }

        const float q = travelCoordinate(road, vehicle);
        vehicle.s = roadCoordinateFromTravel(road, vehicle.laneId, q - amount);
    };

    auto chooseVehicleToProject = [&](std::size_t lhs, std::size_t rhs) {
        const Vehicle& a = proposed[lhs];
        const Vehicle& b = proposed[rhs];

        // Same lane is a 1D ordered-chain problem: always project the follower
        // behind the leader, never push the leader forward.
        if (a.roadId == b.roadId && a.laneId == b.laneId && a.roadId < network.roads.size()) {
            const RoadSegment& road = network.roads[a.roadId];
            if (road.length > 0.0f) {
                const float qa = travelCoordinate(road, a);
                const float qb = travelCoordinate(road, b);
                const float aToB = wrapDistance(qb - qa, road.length);
                const float bToA = wrapDistance(qa - qb, road.length);
                return aToB <= bToA ? lhs : rhs;
            }
        }

        if (a.crossroadEngaged != b.crossroadEngaged) {
            return a.crossroadEngaged ? rhs : lhs;
        }

        const bool aNearCrossroad = isNearCrossroad(simulation, a);
        const bool bNearCrossroad = isNearCrossroad(simulation, b);

        // If one vehicle is already in/near a conflict zone and the other is
        // not, the outside vehicle is the one that must yield. This keeps a car
        // that has engaged a crossroad moving through unless directly blocked.
        if (aNearCrossroad != bNearCrossroad) {
            return aNearCrossroad ? rhs : lhs;
        }

        // Prefer preserving the vehicle that has advanced more this frame.
        // Projecting the lower-progress vehicle backward avoids update-order
        // swaps at crossings and keeps the result deterministic.
        auto progress = [&](std::size_t index) {
            if (index >= current.size()) return 0.0f;
            const Vehicle& before = current[index];
            const Vehicle& after = proposed[index];
            if (before.roadId != after.roadId || before.laneId != after.laneId || after.roadId >= network.roads.size()) {
                return after.speed * dt;
            }
            const RoadSegment& road = network.roads[after.roadId];
            if (road.length <= 0.0f) return after.speed * dt;
            return wrapDistance(travelCoordinate(road, after) - travelCoordinate(road, before), road.length);
        };

        const float aProgress = progress(lhs);
        const float bProgress = progress(rhs);
        if (std::fabs(aProgress - bProgress) > 0.001f) {
            return aProgress < bProgress ? lhs : rhs;
        }

        if (std::fabs(a.speed - b.speed) > 0.001f) {
            return a.speed < b.speed ? lhs : rhs;
        }

        return lhs > rhs ? lhs : rhs;
    };

    constexpr int maxProjectionIterations = 12;
    constexpr float projectionSlop = 0.02f;

    for (int iteration = 0; iteration < maxProjectionIterations; ++iteration) {
        bool changed = false;

        for (std::size_t i = 0; i < proposed.size(); ++i) {
            for (std::size_t j = i + 1; j < proposed.size(); ++j) {
                const Vec2 pi = positionOf(proposed[i]);
                const Vec2 pj = positionOf(proposed[j]);
                const float requiredDistance = vehicleClearanceDistance(proposed[i], proposed[j], minimumCenterDistance);
                const float distanceSq = (pi - pj).lengthSquared();

                if (distanceSq >= requiredDistance * requiredDistance) continue;

                const float distance = std::sqrt(std::max(0.0001f, distanceSq));
                const float penetration = requiredDistance - distance + projectionSlop;
                const std::size_t projectedIndex = chooseVehicleToProject(i, j);

                moveBackwardAlongLane(proposed[projectedIndex], penetration);
                proposed[projectedIndex].speed = std::min(proposed[projectedIndex].speed, proposed[projectedIndex == i ? j : i].speed);
                proposed[projectedIndex].acceleration =
                    (proposed[projectedIndex].speed - current[projectedIndex].speed) / std::max(0.05f, dt);
                changed = true;
            }
        }

        if (!changed) break;
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
    enforceGlobalFootprintProjection(simulation, current, proposed, minimumCenterDistance, dt);
}

} // namespace traffic_flow
