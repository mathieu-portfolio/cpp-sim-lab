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
    // Raw points painted by the user. They are editing data, not the drivable curve.
    std::vector<Vec2> controlPoints;

    // Smoothed closed-loop points generated from controlPoints and used for
    // all sampling, tangent, crossroad and vehicle calculations. Keeping this
    // separate prevents brush spacing/corner artifacts from affecting traffic.
    std::vector<Vec2> drivePoints;

    std::vector<Lane> lanes;
    std::vector<float> arcLengthCache;
    float length = 0.0f;
    std::optional<RoadConnection> startConnection;
    std::optional<RoadConnection> endConnection;
};

struct CrossroadApproach {
    std::size_t roadId = 0;
    float s = 0.0f;
};

struct Crossroad {
    Vec2 position{};
    std::vector<CrossroadApproach> approaches;
};

struct RoadNetwork {
    std::vector<RoadSegment> roads;
    std::vector<Crossroad> crossroads;
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
    float crossroadYieldLookahead = 55.0f;
    float crossroadStopRadius = 11.0f;
    float crossroadClearDelay = 0.45f;
    float crossroadMaxWait = 2.0f; // Deprecated: right-priority now waits while right traffic is moving.
    float stoppedRightPriorityGrace = 0.45f;
    float spawnCrossroadClearance = 36.0f;
    float spawnMinimumGap = 18.0f;
    float physicsMinimumGap = 3.0f;
    float roadSmoothingIterations = 3.0f;
    float roadSmoothingMinPointDistance = 10.0f;
    int arcLengthSamplesPerSpan = 32;
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
    void clearTraffic();
    void generateTraffic();
    void notifyRoadsEdited();

    SimulationConfig& getConfig() { return m_config; }
    const SimulationConfig& getConfig() const { return m_config; }
    std::vector<Vehicle>& getVehicles() { return m_vehicles; }
    const std::vector<Vehicle>& getVehicles() const { return m_vehicles; }
    RoadNetwork& getRoadNetwork() { return m_network; }
    const RoadNetwork& getRoadNetwork() const { return m_network; }
    const SimulationStats& getStats() const { return m_stats; }

    Vec2 sampleRoadCenter(std::size_t roadId, float s) const;
    Vec2 sampleLanePosition(std::size_t roadId, int laneId, float s) const;
    Vec2 sampleRoadTangent(std::size_t roadId, int laneId, float s) const;

private:
    SimulationConfig m_config;
    RoadNetwork m_network;
    std::vector<Vehicle> m_vehicles;
    SimulationStats m_stats;

    float m_throughputAccumulator = 0.0f;
    std::size_t m_wrapCountAccumulator = 0;
    float m_queueAccumulator = 0.0f;
    float m_queueSamples = 0.0f;
    float m_elapsedTime = 0.0f;

    void resetDefaultRoad();
    void rebuildRoadCaches();
    void rebuildRoadCache(RoadSegment& road);
    void rebuildCrossroads();
    float distanceToCrossroadAlongLane(const Vehicle& vehicle, float crossroadS) const;
    bool hasMovingRightSideThreatAtCrossroad(const Vehicle& vehicle, std::size_t vehicleIndex) const;
    bool isInsideCrossroadSpawnClearance(std::size_t roadId, float s) const;
    float distanceAheadOnLane(const Vehicle& follower, const Vehicle& leader) const;
    const Vehicle* findLeader(const Vehicle& vehicle, std::size_t vehicleIndex, float& outGap) const;
    float idmAcceleration(const Vehicle& vehicle, const Vehicle* leader, float gap) const;
};

} // namespace traffic_flow_cpu
