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
    config.useSpatialGrid = useSpatialGrid;
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
} // namespace

TEST(ParticleSimulationBackends, NaiveBackendResolvesParticleCollisions) {
    Simulation sim{makeCollisionConfig(false)};

    spawnDeterministicPair(sim);
    sim.update(0.0f);

    EXPECT_EQ(sim.getStats().particleCount, 2U);
    EXPECT_EQ(sim.getStats().collisionChecks, 1U);
    EXPECT_EQ(sim.getStats().collisionsResolved, 1U);
}

TEST(ParticleSimulationBackends, GridAndNaiveCollisionBackendsMatchForPair) {
    Simulation gridSim{makeCollisionConfig(true)};
    Simulation naiveSim{makeCollisionConfig(false)};

    spawnDeterministicPair(gridSim);
    spawnDeterministicPair(naiveSim);

    gridSim.update(0.0f);
    naiveSim.update(0.0f);

    const auto& gridParticles = gridSim.getParticles();
    const auto& naiveParticles = naiveSim.getParticles();

    ASSERT_EQ(gridParticles.size(), naiveParticles.size());

    for (std::size_t i = 0; i < gridParticles.size(); ++i) {
        expectParticleNear(gridParticles[i], naiveParticles[i]);
    }

    EXPECT_EQ(gridSim.getStats().collisionChecks, naiveSim.getStats().collisionChecks);
    EXPECT_EQ(gridSim.getStats().collisionsResolved, naiveSim.getStats().collisionsResolved);
}
