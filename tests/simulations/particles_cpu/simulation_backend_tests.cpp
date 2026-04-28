#include "Simulation.hpp"

#include <random/Random.hpp>

#include <gtest/gtest.h>

#include <cstddef>

using namespace particles_cpu;

namespace {
constexpr float Epsilon = 0.0001f;

SimulationConfig makeCollisionConfig(bool useSpatialGrid) {
    SimulationConfig config;
    config.width = 500.0f;
    config.height = 500.0f;
    config.gravity = 0.0f;
    config.damping = 1.0f;
    config.bounce = -1.0f;
    config.restitution = 1.0f;
    config.particleRadius = 50.0f;
    config.spawnCount = 2;
    config.maxParticleCount = 2;
    config.gridCellSize = 128.0f;
    config.cellSize = config.gridCellSize;
    config.execution.useSpatialGrid = useSpatialGrid;
    return config;
}

void spawnDeterministicPair(Simulation& sim) {
    Random::seed(1337);
    sim.spawn(Vec2{250.0f, 250.0f});
}

void expectParticleNear(const Particle& actual, const Particle& expected) {
    EXPECT_NEAR(actual.position.x, expected.position.x, Epsilon);
    EXPECT_NEAR(actual.position.y, expected.position.y, Epsilon);
    EXPECT_NEAR(actual.velocity.x, expected.velocity.x, Epsilon);
    EXPECT_NEAR(actual.velocity.y, expected.velocity.y, Epsilon);
    EXPECT_NEAR(actual.radius, expected.radius, Epsilon);
}

void expectSimParticlesNear(const Simulation& actual, const Simulation& expected) {
    const auto& actualParticles = actual.getParticles();
    const auto& expectedParticles = expected.getParticles();

    ASSERT_EQ(actualParticles.size(), expectedParticles.size());

    for (std::size_t i = 0; i < actualParticles.size(); ++i) {
        expectParticleNear(actualParticles[i], expectedParticles[i]);
    }
}
} // namespace

TEST(ParticleSimulationBackends, GridBackendResolvesParticleCollisions) {
    Simulation sim{makeCollisionConfig(true)};

    spawnDeterministicPair(sim);
    sim.update(0.0f);

    EXPECT_EQ(sim.getStats().particleCount, 2U);
    EXPECT_EQ(sim.getStats().collisionChecks, 1U);
    EXPECT_EQ(sim.getStats().collisionsResolved, 1U);
}

TEST(ParticleSimulationBackends, NaiveBackendResolvesParticleCollisions) {
    Simulation sim{makeCollisionConfig(false)};

    spawnDeterministicPair(sim);
    sim.update(0.0f);

    EXPECT_EQ(sim.getStats().particleCount, 2U);
    EXPECT_EQ(sim.getStats().collisionChecks, 1U);
    EXPECT_EQ(sim.getStats().collisionsResolved, 1U);
}

TEST(ParticleSimulationBackends, DisabledSpatialGridStillChecksCollisions) {
    Simulation sim{makeCollisionConfig(false)};

    spawnDeterministicPair(sim);
    sim.update(0.0f);

    EXPECT_FALSE(sim.getConfig().execution.useSpatialGrid);
    EXPECT_GT(sim.getStats().collisionChecks, 0U);
    EXPECT_GT(sim.getStats().collisionsResolved, 0U);
}

TEST(ParticleSimulationBackends, GridAndNaiveCollisionBackendsMatchForPair) {
    Simulation gridSim{makeCollisionConfig(true)};
    Simulation naiveSim{makeCollisionConfig(false)};

    spawnDeterministicPair(gridSim);
    spawnDeterministicPair(naiveSim);

    gridSim.update(0.0f);
    naiveSim.update(0.0f);

    expectSimParticlesNear(gridSim, naiveSim);

    EXPECT_EQ(gridSim.getStats().collisionChecks, naiveSim.getStats().collisionChecks);
    EXPECT_EQ(gridSim.getStats().collisionsResolved, naiveSim.getStats().collisionsResolved);
}

TEST(ParticleSimulationBackends, GridAndNaiveStatsRemainSaneAcrossMultipleUpdates) {
    Simulation gridSim{makeCollisionConfig(true)};
    Simulation naiveSim{makeCollisionConfig(false)};

    spawnDeterministicPair(gridSim);
    spawnDeterministicPair(naiveSim);

    for (int step = 0; step < 3; ++step) {
        gridSim.update(1.0f / 120.0f);
        naiveSim.update(1.0f / 120.0f);

        EXPECT_EQ(gridSim.getStats().particleCount, 2U);
        EXPECT_EQ(naiveSim.getStats().particleCount, 2U);
        EXPECT_LE(gridSim.getStats().collisionsResolved, gridSim.getStats().collisionChecks);
        EXPECT_LE(naiveSim.getStats().collisionsResolved, naiveSim.getStats().collisionChecks);
    }
}

TEST(ParticleSimulationBackends, SwitchingFromGridToNaiveClearsDebugGrid) {
    Simulation sim{makeCollisionConfig(true)};

    spawnDeterministicPair(sim);
    sim.update(0.0f);

    ASSERT_FALSE(sim.getGrid().getCells().empty());

    sim.getConfig().execution.useSpatialGrid = false;
    sim.update(0.0f);

    EXPECT_TRUE(sim.getGrid().getCells().empty());
    EXPECT_EQ(sim.getStats().particleCount, 2U);
    EXPECT_GT(sim.getStats().collisionChecks, 0U);
}

TEST(ParticleSimulationBackends, EmptyNaiveBackendLeavesEmptyGridAndZeroCollisionStats) {
    Simulation sim{makeCollisionConfig(false)};

    sim.update(0.0f);

    EXPECT_TRUE(sim.getGrid().getCells().empty());
    EXPECT_EQ(sim.getStats().particleCount, 0U);
    EXPECT_EQ(sim.getStats().collisionChecks, 0U);
    EXPECT_EQ(sim.getStats().collisionsResolved, 0U);
}
