#include "BoidSteering.hpp"

#include <cmath>

namespace boids {
namespace {
constexpr float Epsilon = 0.0001f;
constexpr float Pi = 3.14159265358979323846f;

float deterministicNoise(std::size_t index, float x, float y) {
    const float seed = static_cast<float>((index * 747796405u) + 2891336453u);
    const float value = std::sin(x * 12.9898f + y * 78.233f + seed * 0.000001f) * 43758.5453f;
    return value - std::floor(value);
}

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

Vec2 computeWander(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    float maxSpeed,
    float jitter
) {
    const Boid& boid = boids[boidIndex];
    const Vec2 forward = boid.velocity.length() > Epsilon
        ? boid.velocity.normalized()
        : Vec2{1.0f, 0.0f};

    const float noise = deterministicNoise(
        boidIndex,
        boid.position.x + boid.velocity.x,
        boid.position.y + boid.velocity.y
    );

    const float angle = (noise * 2.0f - 1.0f) * Pi * jitter;
    const float c = std::cos(angle);
    const float s = std::sin(angle);

    const Vec2 desiredDirection{
        forward.x * c - forward.y * s,
        forward.x * s + forward.y * c
    };

    return steerToward(desiredDirection, boid.velocity, maxSpeed);
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

} // namespace boids
