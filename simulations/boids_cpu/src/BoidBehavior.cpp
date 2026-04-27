#include "BoidBehavior.hpp"

namespace {
constexpr float Epsilon = 0.0001f;

Vec2 steerToward(Vec2 desiredDirection, Vec2 currentVelocity, float maxSpeed) {
    if (desiredDirection.length() <= Epsilon) {
        return {};
    }

    Vec2 desiredVelocity = desiredDirection.normalized() * maxSpeed;
    return desiredVelocity - currentVelocity;
}
}

Vec2 computeAlignment(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& neighbors,
    float maxSpeed
) {
    const Boid& boid = boids[boidIndex];

    Vec2 avgVel{};
    int count = 0;

    for (std::size_t idx : neighbors) {
        if (idx == boidIndex) continue;

        avgVel += boids[idx].velocity;
        ++count;
    }

    if (count == 0) return {};

    avgVel *= 1.0f / static_cast<float>(count);
    return steerToward(avgVel, boid.velocity, maxSpeed);
}

Vec2 computeCohesion(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& neighbors,
    float maxSpeed
) {
    const Boid& boid = boids[boidIndex];

    Vec2 center{};
    int count = 0;

    for (std::size_t idx : neighbors) {
        if (idx == boidIndex) continue;

        center += boids[idx].position;
        ++count;
    }

    if (count == 0) return {};

    center *= 1.0f / static_cast<float>(count);
    return steerToward(center - boid.position, boid.velocity, maxSpeed);
}

Vec2 computeSeparation(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& neighbors,
    float maxSpeed
) {
    const Boid& boid = boids[boidIndex];

    Vec2 away{};
    int count = 0;

    for (std::size_t idx : neighbors) {
        if (idx == boidIndex) continue;

        Vec2 offset = boid.position - boids[idx].position;
        float dist = offset.length();

        if (dist <= Epsilon) continue;

        away += offset.normalized() * (1.0f / dist);
        ++count;
    }

    if (count == 0) return {};

    away *= 1.0f / static_cast<float>(count);
    return steerToward(away, boid.velocity, maxSpeed);
}