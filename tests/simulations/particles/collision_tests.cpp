#include "ParticlePhysics.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace particles;

namespace {
bool nearlyEqual(float a, float b, float epsilon = 0.0001f) {
    return std::fabs(a - b) <= epsilon;
}
}

TEST(ParticleCollision, NonOverlappingParticlesDoNotResolve) {
    SimulationConfig config;

    Particle a{{10.0f, 10.0f}, {}, 4.0f};
    Particle b{{30.0f, 10.0f}, {}, 4.0f};

    const bool resolved = resolveParticleCollision(a, b, config);

    EXPECT_FALSE(resolved);
    EXPECT_TRUE(nearlyEqual(a.position.x, 10.0f));
    EXPECT_TRUE(nearlyEqual(b.position.x, 30.0f));
}

TEST(ParticleCollision, OverlappingParticlesAreSeparated) {
    SimulationConfig config;

    Particle a{{10.0f, 10.0f}, {}, 4.0f};
    Particle b{{16.0f, 10.0f}, {}, 4.0f};

    const bool resolved = resolveParticleCollision(a, b, config);

    EXPECT_TRUE(resolved);

    const float distance = (b.position - a.position).length();
    EXPECT_TRUE(nearlyEqual(distance, 8.0f));
}

TEST(ParticleCollision, HeadOnParticlesBounceApart) {
    SimulationConfig config;
    config.restitution = 1.0f;

    Particle a{{10.0f, 10.0f}, {10.0f, 0.0f}, 4.0f};
    Particle b{{16.0f, 10.0f}, {-10.0f, 0.0f}, 4.0f};

    const bool resolved = resolveParticleCollision(a, b, config);

    EXPECT_TRUE(resolved);
    EXPECT_LT(a.velocity.x, 0.0f);
    EXPECT_GT(b.velocity.x, 0.0f);
}

TEST(ParticleBounds, LeftBoundaryClampsPositionAndReversesVelocity) {
    SimulationConfig config;
    config.width = 100.0f;
    config.height = 100.0f;
    config.bounce = -0.5f;

    Particle p{{-5.0f, 50.0f}, {-10.0f, 0.0f}, 4.0f};

    solveParticleBounds(p, config);

    EXPECT_TRUE(nearlyEqual(p.position.x, 4.0f));
    EXPECT_GT(p.velocity.x, 0.0f);
}

TEST(ParticleBounds, BottomBoundaryClampsPositionAndReversesVelocity) {
    SimulationConfig config;
    config.width = 100.0f;
    config.height = 100.0f;
    config.bounce = -0.5f;

    Particle p{{50.0f, 120.0f}, {0.0f, 10.0f}, 4.0f};

    solveParticleBounds(p, config);

    EXPECT_TRUE(nearlyEqual(p.position.y, 96.0f));
    EXPECT_LT(p.velocity.y, 0.0f);
}