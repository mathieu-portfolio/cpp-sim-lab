#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random/Random.hpp>

namespace traffic_flow_cpu {

namespace {

using LaneIndexLists = std::vector<std::vector<std::size_t>>;

std::size_t laneDirectionBucket(int lane, int laneCount, int direction) {
    const std::size_t laneIndex = static_cast<std::size_t>(lane);
    const std::size_t directionOffset = (direction >= 0) ? 0U : static_cast<std::size_t>(laneCount);
    return directionOffset + laneIndex;
}

int directionForLane(int lane) {
    return (lane % 2 == 0) ? 1 : -1;
}

int findClosestLaneWithDirection(int currentLane, int laneCount, int direction) {
    int bestLane = currentLane;
    int bestDistance = std::numeric_limits<int>::max();
    for (int lane = 0; lane < laneCount; ++lane) {
        if (directionForLane(lane) != direction) {
            continue;
        }
        const int distance = std::abs(lane - currentLane);
        if (distance < bestDistance || (distance == bestDistance && lane < bestLane)) {
            bestDistance = distance;
            bestLane = lane;
        }
    }
    return bestLane;
}

LaneIndexLists buildLaneIndexLists(const std::vector<Vehicle>& vehicles, int laneCount) {
    LaneIndexLists laneVehicleIndices(static_cast<std::size_t>(laneCount * 2));
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        laneVehicleIndices[laneDirectionBucket(vehicles[i].lane, laneCount, vehicles[i].direction)].push_back(i);
    }

    for (auto& laneIndices : laneVehicleIndices) {
        std::sort(laneIndices.begin(), laneIndices.end(), [&](std::size_t a, std::size_t b) {
            return vehicles[a].s < vehicles[b].s;
        });
    }

    return laneVehicleIndices;
}

} // namespace

Simulation::Simulation(SimulationConfig config) : m_config(config) {
    reset();
}

void Simulation::reset() {
    m_vehicles.clear();
    m_vehicles.reserve(m_config.vehicleCount);

    for (std::size_t i = 0; i < m_config.vehicleCount; ++i) {
        Vehicle vehicle;
        vehicle.lane = static_cast<int>(i % static_cast<std::size_t>(m_config.laneCount));
        vehicle.direction = directionForLane(vehicle.lane);
        vehicle.s = (static_cast<float>(i) / static_cast<float>(m_config.vehicleCount)) * m_config.roadLength;
        vehicle.speed = Random::range(m_config.spawnSpeedMin, m_config.spawnSpeedMax);
        m_vehicles.push_back(vehicle);
    }

    m_stats = {};
    m_throughputAccumulator = 0.0f;
    m_wrapCountAccumulator = 0;
    m_queueAccumulator = 0.0f;
    m_queueSamples = 0.0f;
}

float Simulation::gapToLeader(
    const std::vector<Vehicle>& vehicles,
    std::size_t followerIndex,
    std::size_t leaderIndex
) const {
    if (followerIndex == leaderIndex) {
        return m_config.roadLength;
    }

    const Vehicle& follower = vehicles[followerIndex];
    const Vehicle& leader = vehicles[leaderIndex];

    float gap = 0.0f;
    if (follower.direction >= 0) {
        gap = leader.s - follower.s;
    } else {
        gap = follower.s - leader.s;
    }

    return std::max(0.5f, gap - leader.length);
}

float Simulation::idmAcceleration(const Vehicle& vehicle, const Vehicle* leader, float gap) const {
    const float freeRoadTerm = std::pow(
        std::max(0.0f, vehicle.speed / std::max(1.0f, m_config.desiredSpeed)),
        m_config.accelerationExponent
    );

    float interactionTerm = 0.0f;
    if (leader != nullptr) {
        const float deltaV = vehicle.speed - leader->speed;
        const float sqrtTerm = std::max(
            0.1f,
            2.0f * std::sqrt(std::max(0.0f, m_config.maxAcceleration * m_config.comfortableBraking))
        );
        const float interaction = std::max(0.0f, vehicle.speed * deltaV / sqrtTerm);
        const float desiredGap = m_config.minimumGap + vehicle.speed * m_config.desiredTimeHeadway + interaction;
        interactionTerm = std::max(0.0f, desiredGap / std::max(0.5f, gap));
        interactionTerm *= interactionTerm;
    }

    return m_config.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

std::pair<std::size_t, std::size_t> Simulation::findNeighborsInLane(
    const std::vector<Vehicle>& vehicles,
    const LaneIndexLists& laneVehicleIndices,
    std::size_t vehicleIndex,
    int lane
) const {
    if (lane < 0 || lane >= m_config.laneCount) {
        return {vehicleIndex, vehicleIndex};
    }

    const auto& indices = laneVehicleIndices[laneDirectionBucket(lane, m_config.laneCount, directionForLane(lane))];
    if (indices.empty()) {
        return {vehicleIndex, vehicleIndex};
    }

    const auto comp = [&](std::size_t idx, float position) { return vehicles[idx].s < position; };
    auto it = std::lower_bound(indices.begin(), indices.end(), vehicles[vehicleIndex].s, comp);

    std::size_t leader = vehicleIndex;
    std::size_t follower = vehicleIndex;

    if (vehicles[vehicleIndex].direction >= 0) {
        for (auto forward = it; forward != indices.end(); ++forward) {
            if (*forward != vehicleIndex) { leader = *forward; break; }
        }
        for (auto backward = it; backward != indices.begin();) {
            --backward;
            if (*backward != vehicleIndex) { follower = *backward; break; }
        }
    } else {
        for (auto backward = it; backward != indices.begin();) {
            --backward;
            if (*backward != vehicleIndex) { leader = *backward; break; }
        }
        for (auto forward = it; forward != indices.end(); ++forward) {
            if (*forward != vehicleIndex) { follower = *forward; break; }
        }
    }

    return {leader, follower};
}

bool Simulation::shouldChangeLane(
    const std::vector<Vehicle>& vehicles,
    const LaneIndexLists& laneVehicleIndices,
    std::size_t vehicleIndex,
    int targetLane
) const {
    if (targetLane < 0 || targetLane >= m_config.laneCount) {
        return false;
    }
    if (directionForLane(targetLane) != directionForLane(vehicles[vehicleIndex].lane)) {
        return false;
    }

    const Vehicle& vehicle = vehicles[vehicleIndex];
    const auto [currentLeaderIndex, currentFollowerIndex] =
        findNeighborsInLane(vehicles, laneVehicleIndices, vehicleIndex, vehicle.lane);
    const float currentGap = gapToLeader(vehicles, vehicleIndex, currentLeaderIndex);
    const float currentAcc = idmAcceleration(
        vehicle,
        (currentLeaderIndex == vehicleIndex) ? nullptr : &vehicles[currentLeaderIndex],
        currentGap
    );

    const auto [targetLeaderIndex, targetFollowerIndex] =
        findNeighborsInLane(vehicles, laneVehicleIndices, vehicleIndex, targetLane);
    const float targetGap = gapToLeader(vehicles, vehicleIndex, targetLeaderIndex);
    const float targetAcc = idmAcceleration(
        vehicle,
        (targetLeaderIndex == vehicleIndex) ? nullptr : &vehicles[targetLeaderIndex],
        targetGap
    );

    if (targetAcc < currentAcc + m_config.laneChangeThreshold) {
        return false;
    }

    if (targetFollowerIndex != vehicleIndex) {
        const Vehicle& targetFollower = vehicles[targetFollowerIndex];
        const float oldGap = gapToLeader(vehicles, targetFollowerIndex, targetLeaderIndex);
        const float oldAcc = idmAcceleration(
            targetFollower,
            (targetLeaderIndex == targetFollowerIndex) ? nullptr : &vehicles[targetLeaderIndex],
            oldGap
        );
        const float newGap = gapToLeader(vehicles, targetFollowerIndex, vehicleIndex);
        const float newAcc = idmAcceleration(targetFollower, &vehicle, newGap);
        if (newAcc < -m_config.safeDecelerationLimit || (newAcc - oldAcc) < -m_config.laneChangeThreshold) {
            return false;
        }
    }

    if (currentFollowerIndex != vehicleIndex) {
        const Vehicle& currentFollower = vehicles[currentFollowerIndex];
        const float oldGap = gapToLeader(vehicles, currentFollowerIndex, vehicleIndex);
        const float oldAcc = idmAcceleration(currentFollower, &vehicle, oldGap);
        const float newGap = gapToLeader(vehicles, currentFollowerIndex, currentLeaderIndex);
        const float newAcc = idmAcceleration(
            currentFollower,
            (currentLeaderIndex == currentFollowerIndex) ? nullptr : &vehicles[currentLeaderIndex],
            newGap
        );
        if ((newAcc - oldAcc) < -m_config.laneChangeThreshold) {
            return false;
        }
    }

    return true;
}

void Simulation::update(float dt) {
    if (m_vehicles.empty()) {
        return;
    }

    // Phase 1: current state and lane ordering.
    const std::vector<Vehicle> currentVehicles = m_vehicles;
    const LaneIndexLists currentLaneIndices = buildLaneIndexLists(currentVehicles, m_config.laneCount);

    // Phase 2: lane-change decisions.
    std::vector<Vehicle> laneUpdatedVehicles = currentVehicles;
    for (std::size_t i = 0; i < currentVehicles.size(); ++i) {
        laneUpdatedVehicles[i].direction = directionForLane(laneUpdatedVehicles[i].lane);
        const int leftLane = currentVehicles[i].lane - 1;
        const int rightLane = currentVehicles[i].lane + 1;

        if (shouldChangeLane(currentVehicles, currentLaneIndices, i, leftLane)) {
            laneUpdatedVehicles[i].lane = leftLane;
        } else if (shouldChangeLane(currentVehicles, currentLaneIndices, i, rightLane)) {
            laneUpdatedVehicles[i].lane = rightLane;
        }
    }

    // Phase 3 + 4: lane-updated state and rebuilt lane ordering.
    const LaneIndexLists updatedLaneIndices = buildLaneIndexLists(laneUpdatedVehicles, m_config.laneCount);

    std::vector<std::size_t> leaders(laneUpdatedVehicles.size());
    for (const auto& laneIndices : updatedLaneIndices) {
        if (laneIndices.empty()) {
            continue;
        }
        for (std::size_t k = 0; k < laneIndices.size(); ++k) {
            const std::size_t vehicleIndex = laneIndices[k];
            const bool forward = laneUpdatedVehicles[vehicleIndex].direction >= 0;
            if (forward) {
                leaders[vehicleIndex] = (k + 1 < laneIndices.size()) ? laneIndices[k + 1] : vehicleIndex;
            } else {
                leaders[vehicleIndex] = (k > 0) ? laneIndices[k - 1] : vehicleIndex;
            }
        }
    }

    // Phase 5: motion-integrated state.
    std::vector<Vehicle> next = laneUpdatedVehicles;
    float speedSum = 0.0f;
    std::size_t slowCount = 0;

    for (std::size_t i = 0; i < laneUpdatedVehicles.size(); ++i) {
        const std::size_t leader = leaders[i];
        const float gap = gapToLeader(laneUpdatedVehicles, i, leader);
        const Vehicle* leaderPtr = (leader == i) ? nullptr : &laneUpdatedVehicles[leader];

        const float acc = idmAcceleration(laneUpdatedVehicles[i], leaderPtr, gap);
        next[i].acceleration = acc;
        next[i].speed = std::max(0.0f, laneUpdatedVehicles[i].speed + acc * dt);
        next[i].direction = directionForLane(laneUpdatedVehicles[i].lane);
        next[i].s = laneUpdatedVehicles[i].s + static_cast<float>(next[i].direction) * next[i].speed * dt;

        if (next[i].s >= m_config.roadLength) {
            next[i].lane = findClosestLaneWithDirection(next[i].lane, m_config.laneCount, -1);
            next[i].direction = directionForLane(next[i].lane);
            next[i].s = m_config.roadLength;
            ++m_wrapCountAccumulator;
        } else if (next[i].s <= 0.0f) {
            next[i].lane = findClosestLaneWithDirection(next[i].lane, m_config.laneCount, 1);
            next[i].direction = directionForLane(next[i].lane);
            next[i].s = 0.0f;
            ++m_wrapCountAccumulator;
        }

        speedSum += next[i].speed;
        if (next[i].speed < 2.0f) {
            ++slowCount;
        }
    }

    m_vehicles = std::move(next);

    m_throughputAccumulator += dt;
    m_queueAccumulator += static_cast<float>(slowCount);
    m_queueSamples += 1.0f;

    m_stats.averageSpeed = speedSum / static_cast<float>(m_vehicles.size());
    m_stats.averageQueueLength = m_queueAccumulator / std::max(1.0f, m_queueSamples);

    if (m_throughputAccumulator >= 1.0f) {
        m_stats.throughputPerSecond = static_cast<float>(m_wrapCountAccumulator) / m_throughputAccumulator;
        m_throughputAccumulator = 0.0f;
        m_wrapCountAccumulator = 0;
    }
}

} // namespace traffic_flow_cpu
