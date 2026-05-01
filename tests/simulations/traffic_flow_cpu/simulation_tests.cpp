#include "Simulation.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace traffic_flow_cpu {
namespace {

SimulationConfig baseConfig(std::size_t vehicleCount, int laneCount) {
    SimulationConfig config;
    config.vehicleCount = vehicleCount;
    config.laneCount = laneCount;
    config.roadLength = 100.0f;
    config.spawnSpeedMin = 0.0f;
    config.spawnSpeedMax = 0.0f;
    config.desiredSpeed = 30.0f;
    config.maxAcceleration = 1.2f;
    config.comfortableBraking = 2.0f;
    config.laneChangeThreshold = 0.05f;
    return config;
}

TEST(TrafficFlowCpuSimulation, UpdateKeepsPositionsWithinCircularRoad) {
    Simulation sim{baseConfig(2, 1)};
    auto& vehicles = sim.getVehicles();
    vehicles[0].s = 95.0f;
    vehicles[0].speed = 12.0f;
    vehicles[1].s = 40.0f;
    vehicles[1].speed = 0.0f;

    sim.update(1.0f);

    EXPECT_GE(sim.getVehicles()[0].s, 0.0f);
    EXPECT_LT(sim.getVehicles()[0].s, sim.getConfig().roadLength);
}

TEST(TrafficFlowCpuSimulation, LaneChangeUsesPhasedUpdateAcrossVehicles) {
    Simulation sim{baseConfig(4, 2)};
    auto& vehicles = sim.getVehicles();

    vehicles[0].lane = 0; vehicles[0].s = 5.0f;  vehicles[0].speed = 22.0f;
    vehicles[1].lane = 0; vehicles[1].s = 12.0f; vehicles[1].speed = 4.0f;
    vehicles[2].lane = 1; vehicles[2].s = 55.0f; vehicles[2].speed = 18.0f;
    vehicles[3].lane = 1; vehicles[3].s = 80.0f; vehicles[3].speed = 16.0f;

    sim.update(0.2f);

    EXPECT_EQ(sim.getVehicles()[0].lane, 1);
}

TEST(TrafficFlowCpuSimulation, LaneOrderingWrapAroundSelectsNearestLeader) {
    Simulation sim{baseConfig(3, 1)};
    auto& vehicles = sim.getVehicles();

    vehicles[0].lane = 0; vehicles[0].s = 98.0f; vehicles[0].speed = 15.0f;
    vehicles[1].lane = 0; vehicles[1].s = 10.0f; vehicles[1].speed = 2.0f;
    vehicles[2].lane = 0; vehicles[2].s = 60.0f; vehicles[2].speed = 20.0f;

    sim.update(0.1f);

    EXPECT_LT(sim.getVehicles()[0].acceleration, 0.0f);
    EXPECT_TRUE(std::all_of(sim.getVehicles().begin(), sim.getVehicles().end(), [&](const Vehicle& v) {
        return v.s >= 0.0f && v.s < sim.getConfig().roadLength;
    }));
}

} // namespace
} // namespace traffic_flow_cpu
