#include "BoidBehavior.hpp"

namespace boids_cpu {
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

Vec2 limitLength(Vec2 v, float maxLength) {
    float len = v.length();

    if (len > maxLength && len > 0.0f) {
        return v * (maxLength / len);
    }

    return v;
}

Vec2 wrapPosition(Vec2 pos, float w, float h) {
    if (pos.x < 0.0f) pos.x += w;
    if (pos.x > w) pos.x -= w;
    if (pos.y < 0.0f) pos.y += h;
    if (pos.y > h) pos.y -= h;

    return pos;
}

} // namespace boids_cpu
