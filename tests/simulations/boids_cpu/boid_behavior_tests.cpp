#include "BoidBehavior.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace {
constexpr float Epsilon = 0.001f;

void expectNear(Vec2 actual, Vec2 expected) {
    EXPECT_NEAR(actual.x, expected.x, Epsilon);
    EXPECT_NEAR(actual.y, expected.y, Epsilon);
}
}

TEST(BoidBehaviorTest, LimitLengthDoesNotChangeShortVector) {
    Vec2 result = limitLength(Vec2{3.0f, 4.0f}, 10.0f);

    expectNear(result, Vec2{3.0f, 4.0f});
}

TEST(BoidBehaviorTest, LimitLengthClampsLongVector) {
    Vec2 result = limitLength(Vec2{3.0f, 4.0f}, 2.5f);

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
    std::vector<Boid> boids{
        Boid{Vec2{0.0f, 0.0f}, Vec2{10.0f, 0.0f}}
    };

    std::vector<std::size_t> neighbors{};

    Vec2 result = computeAlignment(0, boids, neighbors, 100.0f);

    expectNear(result, Vec2{});
}

TEST(BoidBehaviorTest, AlignmentSteersTowardNeighborVelocity) {
    std::vector<Boid> boids{
        Boid{Vec2{0.0f, 0.0f}, Vec2{10.0f, 0.0f}},
        Boid{Vec2{10.0f, 0.0f}, Vec2{0.0f, 20.0f}}
    };

    std::vector<std::size_t> neighbors{1};

    Vec2 result = computeAlignment(0, boids, neighbors, 100.0f);

    expectNear(result, Vec2{-10.0f, 100.0f});
}

TEST(BoidBehaviorTest, CohesionSteersTowardNeighborCenter) {
    std::vector<Boid> boids{
        Boid{Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}},
        Boid{Vec2{10.0f, 0.0f}, Vec2{0.0f, 0.0f}}
    };

    std::vector<std::size_t> neighbors{1};

    Vec2 result = computeCohesion(0, boids, neighbors, 100.0f);

    expectNear(result, Vec2{100.0f, 0.0f});
}

TEST(BoidBehaviorTest, SeparationSteersAwayFromCloseNeighbor) {
    std::vector<Boid> boids{
        Boid{Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}},
        Boid{Vec2{10.0f, 0.0f}, Vec2{0.0f, 0.0f}}
    };

    std::vector<std::size_t> neighbors{1};

    Vec2 result = computeSeparation(0, boids, neighbors, 100.0f);

    expectNear(result, Vec2{-100.0f, 0.0f});
}

TEST(BoidBehaviorTest, SeparationIgnoresDistantNeighbors) {
    std::vector<Boid> boids{
        Boid{Vec2{0.0f, 0.0f}, Vec2{0.0f, 0.0f}},
        Boid{Vec2{100.0f, 0.0f}, Vec2{0.0f, 0.0f}}
    };

    std::vector<std::size_t> neighbors{};

    Vec2 result = computeSeparation(0, boids, neighbors, 100.0f);

    expectNear(result, Vec2{});
}