#include "Simulation.hpp"

#include <gtest/gtest.h>

using namespace agents_cpu;

namespace {
SimulationConfig singleAgentConfig() {
    SimulationConfig config;
    config.width = 200.0f;
    config.height = 200.0f;
    config.agentCount = 1;
    config.entityCount = 1;
    config.maxSpeed = 0.0f;
    config.maxForce = 0.0f;
    config.seekWeight = 0.0f;
    config.separationWeight = 0.0f;
    config.obstacleAvoidanceWeight = 0.0f;
    config.obstacleAvoidanceRadius = 20.0f;
    config.obstacleRadius = 5.0f;
    config.gridCellSize = 10.0f;
    return config;
}
} // namespace

TEST(AgentsCpuSimulationTest, SpatialGridBuildsObstacleCells) {
    SimulationConfig config = singleAgentConfig();
    config.useSpatialGrid = true;

    Simulation simulation{config};
    simulation.getEntities()[0].position = Vec2{50.0f, 50.0f};
    simulation.getEntities()[0].velocity = Vec2{};
    simulation.getEntities()[0].target = Vec2{50.0f, 50.0f};

    simulation.addObstacle(Vec2{55.0f, 50.0f});
    simulation.update(0.0f);

    const SimulationStats stats = simulation.getStats();

    EXPECT_EQ(stats.obstacleCount, 1u);
    EXPECT_EQ(stats.occupiedObstacleGridCells, 1u);
    EXPECT_EQ(stats.obstacleCandidates, 1u);
    EXPECT_EQ(stats.obstacleChecks, 1u);
    EXPECT_EQ(stats.obstacleOverlapChecks, 1u);
}

TEST(AgentsCpuSimulationTest, SpatialObstacleQuerySkipsFarObstacles) {
    SimulationConfig config = singleAgentConfig();
    config.useSpatialGrid = true;

    Simulation simulation{config};
    simulation.getEntities()[0].position = Vec2{10.0f, 10.0f};
    simulation.getEntities()[0].velocity = Vec2{};
    simulation.getEntities()[0].target = Vec2{10.0f, 10.0f};

    simulation.addObstacle(Vec2{190.0f, 190.0f});
    simulation.update(0.0f);

    const SimulationStats stats = simulation.getStats();

    EXPECT_EQ(stats.obstacleCount, 1u);
    EXPECT_EQ(stats.obstacleCandidates, 0u);
    EXPECT_EQ(stats.obstacleChecks, 0u);
    EXPECT_EQ(stats.obstacleOverlapChecks, 0u);
}

TEST(AgentsCpuSimulationTest, SpatialObstacleOverlapUsesNearbyCandidatesOnly) {
    SimulationConfig config = singleAgentConfig();
    config.useSpatialGrid = true;

    Simulation simulation{config};
    simulation.getEntities()[0].position = Vec2{10.0f, 10.0f};
    simulation.getEntities()[0].velocity = Vec2{};
    simulation.getEntities()[0].target = Vec2{10.0f, 10.0f};

    simulation.addObstacle(Vec2{12.0f, 10.0f});
    simulation.addObstacle(Vec2{190.0f, 190.0f});
    simulation.update(0.0f);

    const SimulationStats stats = simulation.getStats();

    EXPECT_EQ(stats.obstacleCount, 2u);
    EXPECT_EQ(stats.obstacleCandidates, 1u);
    EXPECT_EQ(stats.obstacleChecks, 1u);
    EXPECT_EQ(stats.obstacleOverlapChecks, 1u);
}

TEST(AgentsCpuSimulationTest, NaiveObstacleQueryChecksEveryObstacle) {
    SimulationConfig config = singleAgentConfig();
    config.useSpatialGrid = false;

    Simulation simulation{config};
    simulation.getEntities()[0].position = Vec2{10.0f, 10.0f};
    simulation.getEntities()[0].velocity = Vec2{};
    simulation.getEntities()[0].target = Vec2{10.0f, 10.0f};

    simulation.addObstacle(Vec2{190.0f, 190.0f});
    simulation.update(0.0f);

    const SimulationStats stats = simulation.getStats();

    EXPECT_EQ(stats.obstacleCount, 1u);
    EXPECT_EQ(stats.obstacleChecks, 1u);
    EXPECT_EQ(stats.obstacleOverlapChecks, 1u);
}
