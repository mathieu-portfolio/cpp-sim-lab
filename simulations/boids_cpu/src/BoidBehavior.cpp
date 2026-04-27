#include "BoidBehavior.hpp"

Vec2 computeAlignment(
    const Boid& boid,
    const std::vector<Boid>& boids,
    float radius
) {
    Vec2 averageVelocity{};
    int neighborCount = 0;

    for (const Boid& other : boids) {
        Vec2 offset = other.position - boid.position;
        float distance = offset.length();

        if (distance <= 0.0f || distance >= radius) {
            continue;
        }

        averageVelocity += other.velocity;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    averageVelocity *= 1.0f / static_cast<float>(neighborCount);

    return averageVelocity - boid.velocity;
}

Vec2 computeCohesion(
    const Boid& boid,
    const std::vector<Boid>& boids,
    float radius
) {
    Vec2 center{};
    int neighborCount = 0;

    for (const Boid& other : boids) {
        Vec2 offset = other.position - boid.position;
        float distance = offset.length();

        if (distance <= 0.0f || distance >= radius) {
            continue;
        }

        center += other.position;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    center *= 1.0f / static_cast<float>(neighborCount);

    return center - boid.position;
}

Vec2 computeSeparation(
    const Boid& boid,
    const std::vector<Boid>& boids,
    float radius
) {
    Vec2 steering{};
    int neighborCount = 0;

    for (const Boid& other : boids) {
        Vec2 offset = boid.position - other.position;
        float distance = offset.length();

        if (distance <= 0.0f || distance >= radius) {
            continue;
        }

        // Closer neighbors push harder.
        steering += offset * (1.0f / distance);
        ++neighborCount;
    }

    if (neighborCount > 0) {
        steering *= 1.0f / static_cast<float>(neighborCount);
    }

    return steering;
}

Vec2 limitLength(Vec2 v, float maxLength) {
    float len = v.length();
    if (len > maxLength && len > 0.0f) {
        return v * (maxLength / len);
    }
    return v;
}

Vec2 wrapPosition(Vec2 pos, float w, float h) {
    if (pos.x < 0) pos.x += w;
    if (pos.x > w) pos.x -= w;
    if (pos.y < 0) pos.y += h;
    if (pos.y > h) pos.y -= h;
    return pos;
}
