#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <random/Random.hpp>

namespace traffic_flow_cpu {

Simulation::Simulation(SimulationConfig config) : m_config(config) {
    reset();
}

void Simulation::reset() {
    m_vehicles.clear();
    m_vehicles.reserve(m_config.vehicleCount);

    for (std::size_t i = 0; i < m_config.vehicleCount; ++i) {
        Vehicle vehicle;
        vehicle.lane = static_cast<int>(i % static_cast<std::size_t>(m_config.laneCount));
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

float Simulation::gapToLeader(std::size_t followerIndex, std::size_t leaderIndex) const {
    if (followerIndex == leaderIndex) {
        return m_config.roadLength;
    }

    float gap = m_vehicles[leaderIndex].s - m_vehicles[followerIndex].s;
    if (gap <= 0.0f) {
        gap += m_config.roadLength;
    }

    return std::max(0.5f, gap - m_vehicles[leaderIndex].length);
}

float Simulation::idmAcceleration(const Vehicle& vehicle, const Vehicle* leader, float gap) const {
    const float freeRoadTerm = std::pow(
        std::max(0.0f, vehicle.speed / std::max(1.0f, m_config.desiredSpeed)),
        m_config.accelerationExponent
    );

    float interactionTerm = 0.0f;
    if (leader != nullptr) {
        const float deltaV = vehicle.speed - leader->speed;
        const float sqrtTerm = 2.0f * std::sqrt(m_config.maxAcceleration * m_config.comfortableBraking);
        const float desiredGap = m_config.minimumGap +
            vehicle.speed * m_config.desiredTimeHeadway +
            (vehicle.speed * deltaV) / std::max(0.1f, sqrtTerm);
        interactionTerm = std::max(0.0f, desiredGap / std::max(0.5f, gap));
        interactionTerm *= interactionTerm;
    }

    return m_config.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

bool Simulation::shouldChangeLane(std::size_t vehicleIndex, int targetLane) const {
    if (targetLane < 0 || targetLane >= m_config.laneCount) {
        return false;
    }

    const Vehicle& vehicle = m_vehicles[vehicleIndex];
    const auto findNeighborsInLane = [&](int lane) {
        std::size_t leader = vehicleIndex;
        std::size_t follower = vehicleIndex;
        float minAhead = std::numeric_limits<float>::max();
        float minBehind = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
            if (i == vehicleIndex || m_vehicles[i].lane != lane) {
                continue;
            }
            float ahead = m_vehicles[i].s - vehicle.s;
            if (ahead <= 0.0f) {
                ahead += m_config.roadLength;
            }
            if (ahead < minAhead) {
                minAhead = ahead;
                leader = i;
            }
            float behind = vehicle.s - m_vehicles[i].s;
            if (behind <= 0.0f) {
                behind += m_config.roadLength;
            }
            if (behind < minBehind) {
                minBehind = behind;
                follower = i;
            }
        }
        return std::pair<std::size_t, std::size_t>{leader, follower};
    };

    const auto [currentLeaderIndex, currentFollowerIndex] = findNeighborsInLane(vehicle.lane);
    const float currentGap = gapToLeader(vehicleIndex, currentLeaderIndex);
    const float currentAcc = idmAcceleration(
        vehicle,
        (currentLeaderIndex == vehicleIndex) ? nullptr : &m_vehicles[currentLeaderIndex],
        currentGap
    );

    const auto [targetLeaderIndex, targetFollowerIndex] = findNeighborsInLane(targetLane);
    const float targetGap = gapToLeader(vehicleIndex, targetLeaderIndex);
    const float targetAcc = idmAcceleration(
        vehicle,
        (targetLeaderIndex == vehicleIndex) ? nullptr : &m_vehicles[targetLeaderIndex],
        targetGap
    );

    if (targetAcc < currentAcc + m_config.laneChangeThreshold) {
        return false;
    }

    if (targetFollowerIndex != vehicleIndex) {
        const Vehicle& targetFollower = m_vehicles[targetFollowerIndex];
        const std::size_t oldLeader = findNeighborsInLane(targetLane).first;
        const float oldGap = gapToLeader(targetFollowerIndex, oldLeader);
        const float oldAcc = idmAcceleration(
            targetFollower,
            (oldLeader == targetFollowerIndex) ? nullptr : &m_vehicles[oldLeader],
            oldGap
        );
        const float newGap = std::max(0.5f, vehicle.s - targetFollower.s > 0.0f ? vehicle.s - targetFollower.s : vehicle.s - targetFollower.s + m_config.roadLength);
        const float newAcc = idmAcceleration(targetFollower, &vehicle, std::max(0.5f, newGap - vehicle.length));
        if (newAcc < -m_config.safeDecelerationLimit || (newAcc - oldAcc) < -m_config.laneChangeThreshold) {
            return false;
        }
    }

    if (currentFollowerIndex != vehicleIndex) {
        const Vehicle& currentFollower = m_vehicles[currentFollowerIndex];
        const float oldGap = gapToLeader(currentFollowerIndex, vehicleIndex);
        const float oldAcc = idmAcceleration(currentFollower, &vehicle, oldGap);
        const float newGap = gapToLeader(currentFollowerIndex, currentLeaderIndex);
        const float newAcc = idmAcceleration(
            currentFollower,
            (currentLeaderIndex == currentFollowerIndex) ? nullptr : &m_vehicles[currentLeaderIndex],
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

    std::vector<Vehicle> next = m_vehicles;

    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        const int leftLane = m_vehicles[i].lane - 1;
        const int rightLane = m_vehicles[i].lane + 1;

        if (shouldChangeLane(i, leftLane)) {
            next[i].lane = leftLane;
        } else if (shouldChangeLane(i, rightLane)) {
            next[i].lane = rightLane;
        }
    }

    m_vehicles = next;

    std::vector<std::vector<std::size_t>> laneVehicleIndices(static_cast<std::size_t>(m_config.laneCount));
    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        laneVehicleIndices[static_cast<std::size_t>(m_vehicles[i].lane)].push_back(i);
    }
    for (auto& laneIndices : laneVehicleIndices) {
        std::sort(laneIndices.begin(), laneIndices.end(), [&](std::size_t a, std::size_t b) {
            return m_vehicles[a].s < m_vehicles[b].s;
        });
    }

    std::vector<std::size_t> leaders(m_vehicles.size());
    for (const auto& laneIndices : laneVehicleIndices) {
        if (laneIndices.empty()) {
            continue;
        }
        if (laneIndices.size() == 1) {
            leaders[laneIndices.front()] = laneIndices.front();
            continue;
        }
        for (std::size_t k = 0; k < laneIndices.size(); ++k) {
            const std::size_t follower = laneIndices[k];
            const std::size_t leader = laneIndices[(k + 1) % laneIndices.size()];
            leaders[follower] = leader;
        }
    }

    float speedSum = 0.0f;
    std::size_t slowCount = 0;

    for (std::size_t i = 0; i < next.size(); ++i) {
        const std::size_t leader = leaders[i];
        const float gap = gapToLeader(i, leader);
        const Vehicle* leaderPtr = (leader == i) ? nullptr : &m_vehicles[leader];

        const float acc = idmAcceleration(m_vehicles[i], leaderPtr, gap);
        next[i].acceleration = acc;
        next[i].speed = std::max(0.0f, m_vehicles[i].speed + acc * dt);
        next[i].s = m_vehicles[i].s + next[i].speed * dt;

        if (next[i].s >= m_config.roadLength) {
            next[i].s -= m_config.roadLength;
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
