#pragma once

#include "Vehicle.hpp"

#include <math/Vec2.hpp>
#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace traffic_flow_cpu {

struct Lane {
    int direction = 1; // +1 moves toward s=end, -1 toward s=0
    float lateralOffset = 0.0f;
};

struct RoadConnection {
    std::size_t roadId = 0;
    int laneId = 0;
};

struct RoadSegment {
    std::vector<Vec2> controlPoints;
    std::vector<Lane> lanes;
    std::vector<float> arcLengthCache;
    float length = 0.0f;
    std::optional<RoadConnection> startConnection;
    std::optional<RoadConnection> endConnection;
};

struct RoadNetwork {
    std::vector<RoadSegment> roads;
};

struct SimulationConfig {
    std::size_t vehicleCount = 70;
    float desiredSpeed = 30.0f;
    float maxAcceleration = 1.2f;
    float comfortableBraking = 2.0f;
    float minimumGap = 2.0f;
    float desiredTimeHeadway = 1.4f;
    float accelerationExponent = 4.0f;
    float laneWidth = 22.0f;
    float spawnSpeedMin = 8.0f;
    float spawnSpeedMax = 22.0f;
    int arcLengthSamplesPerSpan = 24;
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
    RoadNetwork& getRoadNetwork() { return m_network; }
    const RoadNetwork& getRoadNetwork() const { return m_network; }
    const SimulationStats& getStats() const { return m_stats; }

    Vec2 sampleRoadCenter(std::size_t roadId, float s) const;
    Vec2 sampleLanePosition(std::size_t roadId, int laneId, float s) const;

private:
    SimulationConfig m_config;
    RoadNetwork m_network;
    std::vector<Vehicle> m_vehicles;
    SimulationStats m_stats;

    float m_throughputAccumulator = 0.0f;
    std::size_t m_wrapCountAccumulator = 0;
    float m_queueAccumulator = 0.0f;
    float m_queueSamples = 0.0f;

    void rebuildRoadCache(RoadSegment& road);
    float idmAcceleration(const Vehicle& vehicle, const Vehicle* leader, float gap) const;
};

} // namespace traffic_flow_cpu
