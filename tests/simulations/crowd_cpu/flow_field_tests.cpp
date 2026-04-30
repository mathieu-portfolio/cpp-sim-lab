#include "Simulation.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace crowd_cpu {
namespace {

SimulationConfig baseConfig(std::size_t agentCount = 1) {
    SimulationConfig config;
    config.width = 120.0f;
    config.height = 120.0f;
    config.agentCount = agentCount;
    config.entityCount = agentCount;
    config.gridCellSize = 20.0f;
    config.maxSpeed = 80.0f;
    config.maxForce = 200.0f;
    config.separationWeight = 0.0f;
    config.obstacleAvoidanceWeight = 0.0f;
    config.execution.useSpatialGrid = true;
    config.execution.useParallelUpdate = false;
    return config;
}

std::size_t cellIndex(const SimulationConfig& config, Vec2 point) {
    const std::size_t gridWidth = static_cast<std::size_t>(config.width / config.gridCellSize) + 1;
    const std::size_t gridHeight = static_cast<std::size_t>(config.height / config.gridCellSize) + 1;
    const auto x = static_cast<std::size_t>(std::clamp(
        static_cast<int>(point.x / config.gridCellSize),
        0,
        static_cast<int>(gridWidth - 1)
    ));
    const auto y = static_cast<std::size_t>(std::clamp(
        static_cast<int>(point.y / config.gridCellSize),
        0,
        static_cast<int>(gridHeight - 1)
    ));
    return y * gridWidth + x;
}

TEST(CrowdCpuFlowField, BlockedCellsBecomeUnreachableInIntegrationField) {
    SimulationConfig config = baseConfig();
    Simulation sim{config};

    sim.setGoal(Vec2{110.0f, 110.0f});
    sim.obstacleMask().paintCircle(Vec2{50.0f, 50.0f}, 10.0f, simfw::simulation::ObstaclePaintMode::Block);

    sim.update(1.0f / 60.0f);

    const auto& integration = sim.getIntegrationField();
    const std::size_t blocked = cellIndex(config, Vec2{50.0f, 50.0f});

    ASSERT_LT(blocked, integration.size());
    EXPECT_FALSE(std::isfinite(integration[blocked]));
}

TEST(CrowdCpuFlowField, SampledFlowRoutesAroundBlockedCells) {
    SimulationConfig config = baseConfig();
    Simulation sim{config};

    sim.setGoal(Vec2{110.0f, 110.0f});
    sim.obstacleMask().paintCircle(Vec2{50.0f, 50.0f}, 10.0f, simfw::simulation::ObstaclePaintMode::Block);

    sim.update(1.0f / 60.0f);

    const Vec2 blockedFlow = sim.sampleFlow(Vec2{50.0f, 50.0f});
    EXPECT_LE(blockedFlow.length(), 1e-4f);

    const Vec2 neighborFlow = sim.sampleFlow(Vec2{30.0f, 50.0f});
    EXPECT_GT(neighborFlow.length(), 0.1f);
}

TEST(CrowdCpuFlowField, AgentMotionFollowsFlowInsteadOfDirectGoalVector) {
    SimulationConfig config = baseConfig();
    config.flowWeight = 1.0f;
    Simulation sim{config};

    sim.setGoal(Vec2{110.0f, 110.0f});
    sim.obstacleMask().paintCircle(Vec2{50.0f, 50.0f}, 10.0f, simfw::simulation::ObstaclePaintMode::Block);

    auto& entities = sim.getEntities();
    ASSERT_EQ(entities.size(), 1u);
    entities[0].position = Vec2{10.0f, 50.0f};
    entities[0].velocity = Vec2{};

    sim.update(1.0f / 30.0f);

    const Agent& agent = sim.getEntities()[0];
    const Vec2 toGoal = (sim.getGoal() - Vec2{10.0f, 50.0f}).normalized();
    const float directness = Vec2::dot(agent.velocity.normalized(), toGoal);

    EXPECT_LT(directness, 0.98f);
    EXPECT_GT(agent.velocity.length(), 0.01f);
}

TEST(CrowdCpuFlowField, AgentMoveIsRejectedIfBlockedByMask) {
    SimulationConfig config = baseConfig();
    config.flowWeight = 0.0f;
    config.separationWeight = 0.0f;

    Simulation sim{config};
    sim.obstacleMask().paintCircle(Vec2{52.0f, 50.0f}, 6.0f, simfw::simulation::ObstaclePaintMode::Block);

    auto& entities = sim.getEntities();
    ASSERT_EQ(entities.size(), 1u);
    entities[0].position = Vec2{50.0f, 50.0f};
    entities[0].velocity = Vec2{60.0f, 0.0f};

    sim.update(1.0f / 60.0f);

    const Agent& agent = sim.getEntities()[0];
    EXPECT_NEAR(agent.position.x, 50.0f, 0.01f);
    EXPECT_NEAR(agent.position.y, 50.0f, 0.01f);
}

} // namespace
} // namespace crowd_cpu
