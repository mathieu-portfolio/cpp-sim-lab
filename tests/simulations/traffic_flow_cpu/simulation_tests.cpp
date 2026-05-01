#include "Simulation.hpp"

#include <gtest/gtest.h>

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
    config.maxAcceleration = 0.0f;
    config.comfortableBraking = 2.0f;
    config.laneChangeThreshold = 999.0f;
    return config;
}

TEST(TrafficFlowCpuSimulation, VehicleReversesAtRoadEnd) {
    Simulation sim{baseConfig(1, 1)};
    auto& vehicle = sim.getVehicles()[0];
    vehicle.s = 99.5f;
    vehicle.speed = 10.0f;
    vehicle.direction = 1;

    sim.update(0.1f);

    EXPECT_FLOAT_EQ(sim.getVehicles()[0].s, sim.getConfig().roadLength);
    EXPECT_EQ(sim.getVehicles()[0].direction, -1);
}

TEST(TrafficFlowCpuSimulation, VehicleReversesAtRoadStart) {
    Simulation sim{baseConfig(1, 1)};
    auto& vehicle = sim.getVehicles()[0];
    vehicle.s = 0.5f;
    vehicle.speed = 10.0f;
    vehicle.direction = -1;

    sim.update(0.1f);

    EXPECT_FLOAT_EQ(sim.getVehicles()[0].s, 0.0f);
    EXPECT_EQ(sim.getVehicles()[0].direction, 1);
}

TEST(TrafficFlowCpuSimulation, OppositeDirectionVehiclesAreNotLaneLeaders) {
    Simulation sim{baseConfig(2, 1)};
    auto& vehicles = sim.getVehicles();

    vehicles[0].lane = 0;
    vehicles[0].s = 10.0f;
    vehicles[0].speed = 20.0f;
    vehicles[0].direction = 1;

    vehicles[1].lane = 0;
    vehicles[1].s = 20.0f;
    vehicles[1].speed = 0.0f;
    vehicles[1].direction = -1;

    sim.update(0.1f);

    EXPECT_FLOAT_EQ(sim.getVehicles()[0].speed, 20.0f);
    EXPECT_FLOAT_EQ(sim.getVehicles()[1].speed, 0.0f);
}

TEST(TrafficFlowCpuSimulation, VehicleCountDoesNotChangeAtEndpoints) {
    Simulation sim{baseConfig(3, 1)};
    auto& vehicles = sim.getVehicles();

    vehicles[0].s = 99.9f; vehicles[0].speed = 5.0f; vehicles[0].direction = 1;
    vehicles[1].s = 0.1f;  vehicles[1].speed = 5.0f; vehicles[1].direction = -1;
    vehicles[2].s = 50.0f; vehicles[2].speed = 5.0f; vehicles[2].direction = 1;

    const std::size_t before = sim.getVehicles().size();
    sim.update(0.1f);
    EXPECT_EQ(sim.getVehicles().size(), before);
}

} // namespace
} // namespace traffic_flow_cpu
