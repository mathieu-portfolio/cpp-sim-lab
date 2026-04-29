#include "BubblePhysics.hpp"

#include <algorithm>
#include <cmath>

namespace bubbles_cpu {
namespace {
constexpr float Epsilon = 0.0001f;
}

void solveBubbleBounds(Bubble& bubble, const SimulationConfig& config) {
    if (bubble.position.x - bubble.radius < 0.0f) {
        bubble.position.x = bubble.radius;
        bubble.velocity.x = std::max(0.0f, -bubble.velocity.x * config.wallBounce);
    }

    if (bubble.position.x + bubble.radius > config.width) {
        bubble.position.x = config.width - bubble.radius;
        bubble.velocity.x = std::min(0.0f, -bubble.velocity.x * config.wallBounce);
    }

    if (bubble.position.y - bubble.radius < 0.0f) {
        bubble.position.y = bubble.radius;
        bubble.velocity.y = std::max(0.0f, -bubble.velocity.y * config.wallBounce);
    }

    if (bubble.position.y + bubble.radius > config.height) {
        bubble.position.y = config.height - bubble.radius;
        bubble.velocity.y = std::min(0.0f, -bubble.velocity.y * config.wallBounce);
    }
}

void applySurfaceTension(Bubble& a, Bubble& b, const SimulationConfig& config, float dt) {
    const Vec2 delta = b.position - a.position;
    const float distance = delta.length();
    const float restDistance = (a.radius + b.radius) * 1.15f;

    if (distance < Epsilon || distance > restDistance * 1.8f) {
        return;
    }

    const Vec2 normal = delta * (1.0f / distance);
    const float offset = distance - restDistance;
    const Vec2 pull = normal * (offset * config.surfaceTension * dt * 0.5f);

    a.velocity += pull;
    b.velocity -= pull;
}

bool resolveBubbleCollision(Bubble& a, Bubble& b, const SimulationConfig& config, float dt) {
    Vec2 delta = b.position - a.position;
    float distance = delta.length();
    const float minDistance = a.radius + b.radius;

    if (distance >= minDistance) {
        return false;
    }

    Vec2 normal = distance < Epsilon ? Vec2{1.0f, 0.0f} : delta * (1.0f / distance);
    if (distance < Epsilon) {
        distance = Epsilon;
    }

    const float overlap = minDistance - distance;
    const Vec2 separation = normal * (overlap * 0.5f);
    a.position -= separation;
    b.position += separation;

    const Vec2 relativeVelocity = b.velocity - a.velocity;
    const float approaching = Vec2::dot(relativeVelocity, normal);
    if (approaching < 0.0f) {
        const Vec2 impulse = normal * (-approaching * config.collisionStiffness * dt * 0.5f);
        a.velocity -= impulse;
        b.velocity += impulse;
    }

    const float normalizedOverlap = overlap / std::max(minDistance, Epsilon);
    a.stress = std::max(a.stress, normalizedOverlap);
    b.stress = std::max(b.stress, normalizedOverlap);

    return true;
}

Bubble mergeBubbles(const Bubble& a, const Bubble& b) {
    const float areaA = a.radius * a.radius;
    const float areaB = b.radius * b.radius;
    const float totalArea = areaA + areaB;

    const float blendA = areaA / std::max(totalArea, Epsilon);
    const float blendB = areaB / std::max(totalArea, Epsilon);

    Bubble merged;
    merged.position = a.position * blendA + b.position * blendB;
    merged.velocity = a.velocity * blendA + b.velocity * blendB;
    merged.radius = std::sqrt(totalArea);
    merged.restRadius = std::sqrt(a.restRadius * a.restRadius + b.restRadius * b.restRadius);
    merged.age = std::max(a.age, b.age);
    merged.stress = 0.0f;
    return merged;
}

} // namespace bubbles_cpu
