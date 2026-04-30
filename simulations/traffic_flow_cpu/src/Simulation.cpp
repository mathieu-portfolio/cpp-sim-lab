#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

std::size_t Simulation::findLeaderIndex(std::size_t vehicleIndex, int lane) const {
    const Vehicle& vehicle = m_vehicles[vehicleIndex];
    float bestDelta = std::numeric_limits<float>::max();
    std::size_t bestIndex = vehicleIndex;

    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        if (i == vehicleIndex || m_vehicles[i].lane != lane) {
            continue;
        }

        float delta = m_vehicles[i].s - vehicle.s;
        if (delta <= 0.0f) {
            delta += m_config.roadLength;
        }

        if (delta < bestDelta) {
            bestDelta = delta;
            bestIndex = i;
        }
    }

    return bestIndex;
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
        interactionTerm = (desiredGap / std::max(0.5f, gap));
        interactionTerm *= interactionTerm;
    }

    return m_config.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

bool Simulation::shouldChangeLane(std::size_t vehicleIndex, int targetLane) const {
    if (targetLane < 0 || targetLane >= m_config.laneCount) {
        return false;
    }

    const Vehicle& vehicle = m_vehicles[vehicleIndex];
    const std::size_t currentLeaderIndex = findLeaderIndex(vehicleIndex, vehicle.lane);
    const float currentGap = gapToLeader(vehicleIndex, currentLeaderIndex);
    const float currentAcc = idmAcceleration(vehicle, &m_vehicles[currentLeaderIndex], currentGap);

    const std::size_t targetLeaderIndex = findLeaderIndex(vehicleIndex, targetLane);
    const float targetGap = gapToLeader(vehicleIndex, targetLeaderIndex);
    const float targetAcc = idmAcceleration(vehicle, &m_vehicles[targetLeaderIndex], targetGap);

    if (targetAcc < currentAcc + m_config.laneChangeThreshold) {
        return false;
    }

    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        if (i == vehicleIndex || m_vehicles[i].lane != targetLane) {
            continue;
        }

        float gapBehind = vehicle.s - m_vehicles[i].s;
        if (gapBehind <= 0.0f) {
            gapBehind += m_config.roadLength;
        }

        if (gapBehind < 24.0f) {
            const float followerAcc = idmAcceleration(m_vehicles[i], &vehicle, std::max(0.5f, gapBehind - vehicle.length));
            if (followerAcc < -m_config.safeDecelerationLimit) {
                return false;
            }
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

    float speedSum = 0.0f;
    std::size_t slowCount = 0;

    for (std::size_t i = 0; i < next.size(); ++i) {
        const std::size_t leader = findLeaderIndex(i, next[i].lane);
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
