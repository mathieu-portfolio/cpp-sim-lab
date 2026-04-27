#include "BoidBehavior.hpp"

namespace {
constexpr float Epsilon = 0.0001f;

bool isSelf(const Boid& a, const Boid& b) {
    return &a == &b;
}

bool isWithinRadius(float distance, float radius) {
    return distance > Epsilon && distance < radius;
}

Vec2 steerToward(Vec2 desiredDirection, Vec2 currentVelocity, float maxSpeed) {
    if (desiredDirection.length() <= Epsilon) {
        return {};
    }

    Vec2 desiredVelocity = desiredDirection.normalized() * maxSpeed;
    return desiredVelocity - currentVelocity;
}
}

Vec2 computeAlignment(
    const Boid& boid,
    const std::vector<Boid>& boids,
    float radius,
    float maxSpeed
) {
    Vec2 averageVelocity{};
    int neighborCount = 0;

    for (const Boid& other : boids) {
        if (isSelf(boid, other)) {
            continue;
        }

        float distance = (other.position - boid.position).length();

        if (!isWithinRadius(distance, radius)) {
            continue;
        }

        averageVelocity += other.velocity;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    averageVelocity *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(averageVelocity, boid.velocity, maxSpeed);
}

Vec2 computeCohesion(
    const Boid& boid,
    const std::vector<Boid>& boids,
    float radius,
    float maxSpeed
) {
    Vec2 center{};
    int neighborCount = 0;

    for (const Boid& other : boids) {
        if (isSelf(boid, other)) {
            continue;
        }

        float distance = (other.position - boid.position).length();

        if (!isWithinRadius(distance, radius)) {
            continue;
        }

        center += other.position;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    center *= 1.0f / static_cast<float>(neighborCount);

    Vec2 directionToCenter = center - boid.position;
    return steerToward(directionToCenter, boid.velocity, maxSpeed);
}

Vec2 computeSeparation(
    const Boid& boid,
    const std::vector<Boid>& boids,
    float radius,
    float maxSpeed
) {
    Vec2 away{};
    int neighborCount = 0;

    for (const Boid& other : boids) {
        if (isSelf(boid, other)) {
            continue;
        }

        Vec2 offset = boid.position - other.position;
        float distance = offset.length();

        if (distance >= radius) {
            continue;
        }

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