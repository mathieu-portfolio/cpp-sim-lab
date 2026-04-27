#include "BoidBehavior.hpp"

#include <gtest/gtest.h>

namespace {
constexpr float Epsilon = 0.001f;

void expectNear(Vec2 actual, Vec2 expected) {
    EXPECT_NEAR(actual.x, expected.x, Epsilon);
    EXPECT_NEAR(actual.y, expected.y, Epsilon);
}
}

TEST(BoidBehaviorTest, LimitLengthDoesNotChangeShortVector) {
    Vec2 v{3.0f, 4.0f};

    Vec2 result = limitLength(v, 10.0f);

    expectNear(result, Vec2{3.0f, 4.0f});
}

TEST(BoidBehaviorTest, LimitLengthClampsLongVector) {
    Vec2 v{3.0f, 4.0f};

    Vec2 result = limitLength(v, 2.5f);

    EXPECT_NEAR(result.length(), 2.5f, Epsilon);
}

TEST(BoidBehaviorTest, WrapPositionWrapsLeftEdge) {
    Vec2 result = wrapPosition(Vec2{-1.0f, 50.0f}, 100.0f, 100.0f);

    expectNear(result, Vec2{99.0f, 50.0f});
}

TEST(BoidBehaviorTest, WrapPositionWrapsRightEdge) {
    Vec2 result = wrapPosition(Vec2{101.0f, 50.0f}, 100.0f, 100.0f);

    expectNear(result, Vec2{1.0f, 50.0f});
}

TEST(BoidBehaviorTest, AlignmentReturnsNoSteeringWithoutNeighbors) {
    Boid boid{
        Vec2{0.0f, 0.0f},
        Vec2{10.0f, 0.0f}
    };

    std::vector<Boid> boids{boid};

    Vec2 result = computeAlignment(boid, boids, 50.0f, 100.0f);

    expectNear(result, Vec2{});
}

TEST(BoidBehaviorTest, AlignmentSteersTowardNeighborVelocity) {
    Boid boid{
        Vec2{0.0f, 0.0f},
        Vec2{10.0f, 0.0f}
    };

    Boid neighbor{
        Vec2{10.0f, 0.0f},
        Vec2{0.0f, 20.0f}
    };

    std::vector<Boid> boids{boid, neighbor};

    Vec2 result = computeAlignment(boids[0], boids, 50.0f, 100.0f);

    expectNear(result, Vec2{-10.0f, 100.0f});
}

TEST(BoidBehaviorTest, CohesionSteersTowardNeighborCenter) {
    Boid boid{
        Vec2{0.0f, 0.0f},
        Vec2{0.0f, 0.0f}
    };

    Boid neighbor{
        Vec2{10.0f, 0.0f},
        Vec2{0.0f, 0.0f}
    };

    std::vector<Boid> boids{boid, neighbor};

    Vec2 result = computeCohesion(boids[0], boids, 50.0f, 100.0f);

    expectNear(result, Vec2{100.0f, 0.0f});
}

TEST(BoidBehaviorTest, SeparationSteersAwayFromCloseNeighbor) {
    Boid boid{
        Vec2{0.0f, 0.0f},
        Vec2{0.0f, 0.0f}
    };

    Boid neighbor{
        Vec2{10.0f, 0.0f},
        Vec2{0.0f, 0.0f}
    };

    std::vector<Boid> boids{boid, neighbor};

    Vec2 result = computeSeparation(boids[0], boids, 50.0f, 100.0f);

    expectNear(result, Vec2{-100.0f, 0.0f});
}

TEST(BoidBehaviorTest, SeparationIgnoresDistantNeighbors) {
    Boid boid{
        Vec2{0.0f, 0.0f},
        Vec2{0.0f, 0.0f}
    };

    Boid distant{
        Vec2{100.0f, 0.0f},
        Vec2{0.0f, 0.0f}
    };

    std::vector<Boid> boids{boid, distant};

    Vec2 result = computeSeparation(boids[0], boids, 50.0f, 100.0f);

    expectNear(result, Vec2{});
}