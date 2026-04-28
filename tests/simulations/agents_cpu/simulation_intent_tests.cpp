#include "AgentIntent.hpp"
#include "Simulation.hpp"

#include <gtest/gtest.h>

#include <cstddef>

namespace agents_cpu {
namespace {

SimulationConfig baseConfig(std::size_t agentCount = 8) {
    SimulationConfig config;
    config.width = 800.0f;
    config.height = 800.0f;
    config.agentCount = agentCount;
    config.entityCount = agentCount;
    config.useSpatialGrid = true;
    config.useParallelUpdate = false;
    config.gridCellSize = 64.0f;
    config.maxSpeed = 100.0f;
    config.maxForce = 100.0f;
    config.targetRadius = 4.0f;
    config.obstacleRadius = 16.0f;
    config.obstacleAvoidanceRadius = 64.0f;
    config.obstacleIntentRadius = 2000.0f;
    return config;
}

std::size_t countedIntents(const SimulationStats& stats) {
    return stats.seekingTargetCount +
        stats.avoidingObstacleCount +
        stats.idleCount;
}

TEST(AgentIntent, NamesAreStableForDebugUi) {
    EXPECT_STREQ(intentName(AgentIntent::SeekTarget), "seek target");
    EXPECT_STREQ(intentName(AgentIntent::AvoidObstacle), "avoid obstacle");
    EXPECT_STREQ(intentName(AgentIntent::Idle), "idle");

    EXPECT_TRUE(isMovingIntent(AgentIntent::SeekTarget));
    EXPECT_TRUE(isMovingIntent(AgentIntent::AvoidObstacle));
    EXPECT_FALSE(isMovingIntent(AgentIntent::Idle));
}

TEST(AgentsCpuIntent, ObstacleThreatSwitchesAgentsToAvoidIntent) {
    Simulation sim{baseConfig(16)};
    sim.addObstacle(Vec2{400.0f, 400.0f});

    sim.update(1.0f / 60.0f);

    const SimulationStats& stats = sim.getStats();
    EXPECT_EQ(stats.agentCount, 16u);
    EXPECT_EQ(countedIntents(stats), stats.agentCount);
    EXPECT_EQ(stats.avoidingObstacleCount, stats.agentCount);
    EXPECT_GT(stats.intentChanges, 0u);

    for (const Agent& agent : sim.getAgents()) {
        EXPECT_EQ(agent.intent, AgentIntent::AvoidObstacle);
    }
}

TEST(AgentsCpuIntent, DisabledIntentKeepsAgentsSeekingEvenNearObstacles) {
    SimulationConfig config = baseConfig(12);
    config.useIntent = false;

    Simulation sim{config};
    sim.addObstacle(Vec2{400.0f, 400.0f});

    sim.update(1.0f / 60.0f);

    const SimulationStats& stats = sim.getStats();
    EXPECT_EQ(stats.agentCount, 12u);
    EXPECT_EQ(countedIntents(stats), stats.agentCount);
    EXPECT_EQ(stats.seekingTargetCount, stats.agentCount);
    EXPECT_EQ(stats.avoidingObstacleCount, 0u);
    EXPECT_EQ(stats.idleCount, 0u);

    for (const Agent& agent : sim.getAgents()) {
        EXPECT_EQ(agent.intent, AgentIntent::SeekTarget);
    }
}

TEST(AgentsCpuIntent, AgentsAtTargetBecomeIdleAndDampVelocity) {
    SimulationConfig config = baseConfig(1);
    config.width = 0.0f;
    config.height = 0.0f;
    config.targetRadius = 1.0f;
    config.idleDamping = 1000.0f;
    config.useSpatialGrid = false;

    Simulation sim{config};

    sim.update(1.0f / 60.0f);

    const SimulationStats& stats = sim.getStats();
    ASSERT_EQ(stats.agentCount, 1u);
    EXPECT_EQ(stats.idleCount, 1u);
    EXPECT_EQ(stats.seekingTargetCount, 0u);
    EXPECT_EQ(stats.avoidingObstacleCount, 0u);
    ASSERT_EQ(sim.getAgents().size(), 1u);
    EXPECT_EQ(sim.getAgents()[0].intent, AgentIntent::Idle);
    EXPECT_FLOAT_EQ(sim.getAgents()[0].velocity.x, 0.0f);
    EXPECT_FLOAT_EQ(sim.getAgents()[0].velocity.y, 0.0f);
}

TEST(AgentsCpuIntent, ParallelUpdateAccumulatesIntentStats) {
    SimulationConfig config = baseConfig(512);
    config.useParallelUpdate = true;

    Simulation sim{config};
    sim.addObstacle(Vec2{400.0f, 400.0f});

    sim.update(1.0f / 60.0f);

    const SimulationStats& stats = sim.getStats();
    EXPECT_EQ(stats.agentCount, 512u);
    EXPECT_EQ(countedIntents(stats), stats.agentCount);
    EXPECT_EQ(stats.avoidingObstacleCount, stats.agentCount);
}

} // namespace
} // namespace agents_cpu
