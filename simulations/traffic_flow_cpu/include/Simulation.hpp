#pragma once

#include "Vehicle.hpp"

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>

#include <cstddef>
#include <vector>

namespace traffic_flow_cpu {

struct SimulationConfig {
    std::size_t vehicleCount = 70;
    int laneCount = 2;
    float roadLength = 700.0f;

    float desiredSpeed = 30.0f;
    float maxAcceleration = 1.2f;
    float comfortableBraking = 2.0f;
    float minimumGap = 2.0f;
    float desiredTimeHeadway = 1.4f;
    float accelerationExponent = 4.0f;

    float laneChangeThreshold = 0.2f;
    float safeDecelerationLimit = 2.5f;

    float laneWidth = 22.0f;
    float spawnSpeedMin = 8.0f;
    float spawnSpeedMax = 22.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
};

struct SimulationStats {
    float throughputPerSecond = 0.0f;
    float averageSpeed = 0.0f;
    float averageQueueLength = 0.0f;
};

class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});

    void reset();
    void update(float dt);

    SimulationConfig& getConfig() { return m_config; }
    const SimulationConfig& getConfig() const { return m_config; }
    std::vector<Vehicle>& getVehicles() { return m_vehicles; }
    const std::vector<Vehicle>& getVehicles() const { return m_vehicles; }
    const SimulationStats& getStats() const { return m_stats; }

private:
    SimulationConfig m_config;
    std::vector<Vehicle> m_vehicles;
    SimulationStats m_stats;

    float m_throughputAccumulator = 0.0f;
    std::size_t m_wrapCountAccumulator = 0;
    float m_queueAccumulator = 0.0f;
    float m_queueSamples = 0.0f;

    std::size_t findLeaderIndex(std::size_t vehicleIndex, int lane) const;
    float gapToLeader(std::size_t followerIndex, std::size_t leaderIndex) const;
    float idmAcceleration(const Vehicle& vehicle, const Vehicle* leader, float gap) const;
    bool shouldChangeLane(std::size_t vehicleIndex, int targetLane) const;
};

} // namespace traffic_flow_cpu
