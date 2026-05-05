#include "Simulation.hpp"

#include <gtest/gtest.h>

namespace traffic_flow {
namespace {

TEST(TrafficFlowCpuSimulation, SplineSamplingIsDeterministic) {
    Simulation sim;
    const auto& road = sim.getRoadNetwork().roads[0];
    const Vec2 p0 = sim.sampleRoadCenter(0, road.length * 0.25f);
    const Vec2 p1 = sim.sampleRoadCenter(0, road.length * 0.25f);
    EXPECT_FLOAT_EQ(p0.x, p1.x);
    EXPECT_FLOAT_EQ(p0.y, p1.y);
}

TEST(TrafficFlowCpuSimulation, ArcLengthMovementUsesDistance) {
    SimulationConfig cfg;
    cfg.vehicleCount = 1;
    cfg.maxAcceleration = 0.0f;
    cfg.spawnSpeedMin = 10.0f;
    cfg.spawnSpeedMax = 10.0f;
    Simulation sim{cfg};
    auto& v = sim.getVehicles()[0];
    v.laneId = 0;
    const float start = v.s;
    sim.update(0.5f);
    EXPECT_NEAR(sim.getVehicles()[0].s - start, 5.0f, 0.001f);
}

TEST(TrafficFlowCpuSimulation, LaneOffsetCreatesDistinctPositions) {
    Simulation sim;
    const auto& road = sim.getRoadNetwork().roads[0];
    const float s = road.length * 0.5f;
    const Vec2 left = sim.sampleLanePosition(0, 0, s);
    const Vec2 right = sim.sampleLanePosition(0, 1, s);
    EXPECT_GT((right - left).length(), 1.0f);
}

TEST(TrafficFlowCpuSimulation, LaneDirectionIsFixedByLane) {
    SimulationConfig cfg;
    cfg.vehicleCount = 2;
    cfg.maxAcceleration = 0.0f;
    cfg.spawnSpeedMin = 5.0f;
    cfg.spawnSpeedMax = 5.0f;
    Simulation sim{cfg};
    auto& vehicles = sim.getVehicles();
    vehicles[0].laneId = 0;
    vehicles[1].laneId = 1;
    vehicles[0].s = vehicles[1].s = sim.getRoadNetwork().roads[0].length * 0.5f;
    const float startS0 = vehicles[0].s;
    const float startS1 = vehicles[1].s;
    sim.update(1.0f);
    EXPECT_GT(sim.getVehicles()[0].s, startS0);
    EXPECT_LT(sim.getVehicles()[1].s, startS1);
}

TEST(TrafficFlowCpuSimulation, EndpointConnectionTransfersVehicle) {
    SimulationConfig cfg;
    cfg.vehicleCount = 1;
    cfg.maxAcceleration = 0.0f;
    cfg.spawnSpeedMin = 5.0f;
    cfg.spawnSpeedMax = 5.0f;
    Simulation sim{cfg};
    RoadSegment extra = sim.getRoadNetwork().roads[0];
    sim.getRoadNetwork().roads.push_back(extra);
    sim.getRoadNetwork().roads[0].endConnection = RoadConnection{1, 0};
    auto& v = sim.getVehicles()[0];
    v.roadId = 0;
    v.laneId = 0;
    v.s = sim.getRoadNetwork().roads[0].length - 0.1f;
    sim.update(0.1f);
    EXPECT_EQ(sim.getVehicles()[0].roadId, 1U);
    EXPECT_NEAR(sim.getVehicles()[0].s, 0.0f, 1e-4f);
}

} // namespace
} // namespace traffic_flow
