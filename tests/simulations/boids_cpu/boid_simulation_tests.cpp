#include "Simulation.hpp"

#include <random/Random.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>

using namespace boids_cpu;

namespace {
constexpr float Epsilon = 0.001f;

SimulationConfig deterministicConfig() {
    SimulationConfig config;
    config.width = 512.0f;
    config.height = 512.0f;
    config.boidCount = 96;
    config.entityCount = config.boidCount;
    config.perceptionRadius = 72.0f;
    config.separationRadius = 28.0f;
    config.gridCellSize = 48.0f;
    config.maxSpeed = 120.0f;
    config.maxForce = 80.0f;
    config.wanderWeight = 0.0f;
    return config;
}

Simulation makeSimulation(SimulationConfig config) {
    Random::seed(12345u);
    return Simulation{config};
}

void expectNear(Vec2 actual, Vec2 expected) {
    EXPECT_NEAR(actual.x, expected.x, Epsilon);
    EXPECT_NEAR(actual.y, expected.y, Epsilon);
}

bool differs(Vec2 lhs, Vec2 rhs) {
    return std::abs(lhs.x - rhs.x) > Epsilon ||
        std::abs(lhs.y - rhs.y) > Epsilon;
}

void expectSameBoids(const Simulation& lhs, const Simulation& rhs) {
    const auto& lhsBoids = lhs.getBoids();
    const auto& rhsBoids = rhs.getBoids();

    ASSERT_EQ(lhsBoids.size(), rhsBoids.size());

    for (std::size_t i = 0; i < lhsBoids.size(); ++i) {
        SCOPED_TRACE(i);
        expectNear(lhsBoids[i].position, rhsBoids[i].position);
        expectNear(lhsBoids[i].velocity, rhsBoids[i].velocity);
    }
}

bool anyBoidDiffers(const Simulation& lhs, const Simulation& rhs) {
    const auto& lhsBoids = lhs.getBoids();
    const auto& rhsBoids = rhs.getBoids();

    if (lhsBoids.size() != rhsBoids.size()) {
        return true;
    }

    for (std::size_t i = 0; i < lhsBoids.size(); ++i) {
        if (differs(lhsBoids[i].position, rhsBoids[i].position) ||
            differs(lhsBoids[i].velocity, rhsBoids[i].velocity)) {
            return true;
        }
    }

    return false;
}
} // namespace

TEST(BoidSimulationTest, WanderIsDisabledByDefault) {
    SimulationConfig config;

    EXPECT_FLOAT_EQ(config.wanderWeight, 0.0f);
}

TEST(BoidSimulationTest, EnablingWanderChangesTrajectory) {
    SimulationConfig baselineConfig = deterministicConfig();
    baselineConfig.execution.useSpatialGrid = true;
    baselineConfig.execution.useParallelUpdate = false;

    SimulationConfig wanderConfig = baselineConfig;
    wanderConfig.wanderWeight = 0.75f;

    Simulation baseline = makeSimulation(baselineConfig);
    Simulation wander = makeSimulation(wanderConfig);

    for (int step = 0; step < 3; ++step) {
        baseline.update(1.0f / 60.0f);
        wander.update(1.0f / 60.0f);
    }

    EXPECT_TRUE(anyBoidDiffers(baseline, wander));
}

TEST(BoidSimulationTest, SpatialGridMatchesNaiveBackendForOneStep) {
    SimulationConfig naiveConfig = deterministicConfig();
    naiveConfig.execution.useSpatialGrid = false;
    naiveConfig.execution.useParallelUpdate = false;

    SimulationConfig gridConfig = naiveConfig;
    gridConfig.execution.useSpatialGrid = true;

    Simulation naive = makeSimulation(naiveConfig);
    Simulation grid = makeSimulation(gridConfig);

    naive.update(1.0f / 60.0f);
    grid.update(1.0f / 60.0f);

    expectSameBoids(naive, grid);
}

TEST(BoidSimulationTest, SpatialGridMatchesNaiveBackendAcrossMultipleSteps) {
    SimulationConfig naiveConfig = deterministicConfig();
    naiveConfig.execution.useSpatialGrid = false;
    naiveConfig.execution.useParallelUpdate = false;

    SimulationConfig gridConfig = naiveConfig;
    gridConfig.execution.useSpatialGrid = true;

    Simulation naive = makeSimulation(naiveConfig);
    Simulation grid = makeSimulation(gridConfig);

    for (int step = 0; step < 10; ++step) {
        naive.update(1.0f / 60.0f);
        grid.update(1.0f / 60.0f);
    }

    expectSameBoids(naive, grid);
}

TEST(BoidSimulationTest, ParallelUpdateMatchesSingleThreadBackend) {
    SimulationConfig singleThreadConfig = deterministicConfig();
    singleThreadConfig.execution.useSpatialGrid = true;
    singleThreadConfig.execution.useParallelUpdate = false;

    SimulationConfig parallelConfig = singleThreadConfig;
    parallelConfig.execution.useParallelUpdate = true;

    Simulation singleThread = makeSimulation(singleThreadConfig);
    Simulation parallel = makeSimulation(parallelConfig);

    for (int step = 0; step < 10; ++step) {
        singleThread.update(1.0f / 60.0f);
        parallel.update(1.0f / 60.0f);
    }

    expectSameBoids(singleThread, parallel);

    EXPECT_EQ(
        singleThread.getStats().neighborChecks,
        parallel.getStats().neighborChecks
    );
    EXPECT_EQ(
        singleThread.getStats().neighborCandidates,
        parallel.getStats().neighborCandidates
    );
    EXPECT_EQ(
        singleThread.getStats().occupiedGridCells,
        parallel.getStats().occupiedGridCells
    );
}
