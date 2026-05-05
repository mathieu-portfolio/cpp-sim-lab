#include "BubblePhysics.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace bubbles;

namespace {
bool nearlyEqual(float a, float b, float epsilon = 0.0001f) {
    return std::fabs(a - b) <= epsilon;
}
}

TEST(BubbleMerge, PreservesArea) {
    Bubble a{{0.0f, 0.0f}, {}, 3.0f, 3.0f};
    Bubble b{{0.0f, 0.0f}, {}, 4.0f, 4.0f};

    Bubble merged = mergeBubbles(a, b);
    EXPECT_TRUE(nearlyEqual(merged.radius * merged.radius, 25.0f));
}

TEST(BubbleMerge, VelocityIsAreaWeighted) {
    Bubble a{{0.0f, 0.0f}, {10.0f, 0.0f}, 3.0f, 3.0f};
    Bubble b{{0.0f, 0.0f}, {0.0f, 0.0f}, 4.0f, 4.0f};

    Bubble merged = mergeBubbles(a, b);
    EXPECT_TRUE(nearlyEqual(merged.velocity.x, 3.6f));
}

TEST(BubbleCollision, OverlapIsSeparated) {
    SimulationConfig config;
    Bubble a{{10.0f, 10.0f}, {}, 6.0f, 6.0f};
    Bubble b{{18.0f, 10.0f}, {}, 6.0f, 6.0f};

    EXPECT_TRUE(resolveBubbleCollision(a, b, config, 1.0f / 60.0f));
    EXPECT_TRUE(nearlyEqual((b.position - a.position).length(), 12.0f));
}

TEST(BubbleBurst, EnabledBurstRemovesBubbleByAge) {
    SimulationConfig config;
    config.enableBurst = true;
    config.burstAgeThreshold = 0.0f;
    config.maxBubbleCount = 4;
    config.spawnCount = 1;
    Simulation sim{config};
    sim.spawn({100.0f, 100.0f});
    sim.update(1.0f / 60.0f);
    EXPECT_EQ(sim.getBubbles().size(), 0U);
}
