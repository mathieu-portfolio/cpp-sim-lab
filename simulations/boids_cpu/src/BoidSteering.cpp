#include "BoidSteering.hpp"

namespace boids_cpu {
namespace {
constexpr float Epsilon = 0.0001f;

Vec2 steerToward(Vec2 desiredDirection, Vec2 currentVelocity, float maxSpeed) {
    if (desiredDirection.length() <= Epsilon) {
        return {};
    }

    const Vec2 desiredVelocity = desiredDirection.normalized() * maxSpeed;
    return desiredVelocity - currentVelocity;
}
} // namespace

Vec2 computeAlignment(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    std::span<const std::size_t> neighbors,
    float maxSpeed
) {
    const Boid& boid = boids[boidIndex];

    Vec2 averageVelocity{};
    int neighborCount = 0;

    for (std::size_t neighborIndex : neighbors) {
        if (neighborIndex == boidIndex) {
            continue;
        }

        averageVelocity += boids[neighborIndex].velocity;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    averageVelocity *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(averageVelocity, boid.velocity, maxSpeed);
}

Vec2 computeCohesion(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    std::span<const std::size_t> neighbors,
    float maxSpeed
) {
    const Boid& boid = boids[boidIndex];

    Vec2 center{};
    int neighborCount = 0;

    for (std::size_t neighborIndex : neighbors) {
        if (neighborIndex == boidIndex) {
            continue;
        }

        center += boids[neighborIndex].position;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    center *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(center - boid.position, boid.velocity, maxSpeed);
}

Vec2 computeSeparation(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    std::span<const std::size_t> neighbors,
    float maxSpeed
) {
    const Boid& boid = boids[boidIndex];

    Vec2 away{};
    int neighborCount = 0;

    for (std::size_t neighborIndex : neighbors) {
        if (neighborIndex == boidIndex) {
            continue;
        }

        const Vec2 offset = boid.position - boids[neighborIndex].position;
        const float distance = offset.length();

        if (distance <= Epsilon) {
            continue;
        }

        away += offset.normalized() * (1.0f / distance);
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    away *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(away, boid.velocity, maxSpeed);
}

Vec2 limitLength(Vec2 value, float maxLength) {
    const float length = value.length();

    if (length > maxLength && length > 0.0f) {
        return value * (maxLength / length);
    }

    return value;
}

Vec2 wrapPosition(Vec2 position, float width, float height) {
    if (position.x < 0.0f) {
        position.x += width;
    }

    if (position.x > width) {
        position.x -= width;
    }

    if (position.y < 0.0f) {
        position.y += height;
    }

    if (position.y > height) {
        position.y -= height;
    }

    return position;
}

} // namespace boids_cpu
