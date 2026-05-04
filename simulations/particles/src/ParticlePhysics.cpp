#include "ParticlePhysics.hpp"

namespace particles_cpu {
namespace {
constexpr float Epsilon = 0.0001f;
}

void solveParticleBounds(Particle& p, const SimulationConfig& config) {
    if (p.position.x - p.radius < 0.0f) {
        p.position.x = p.radius;
        if (p.velocity.x < 0.0f) {
            p.velocity.x *= config.bounce;
        }
    }

    if (p.position.x + p.radius > config.width) {
        p.position.x = config.width - p.radius;
        if (p.velocity.x > 0.0f) {
            p.velocity.x *= config.bounce;
        }
    }

    if (p.position.y + p.radius > config.height) {
        p.position.y = config.height - p.radius;
        if (p.velocity.y > 0.0f) {
            p.velocity.y *= config.bounce;
        }
    }

    if (p.position.y - p.radius < 0.0f) {
        p.position.y = p.radius;
        if (p.velocity.y < 0.0f) {
            p.velocity.y *= config.bounce;
        }
    }
}

bool resolveParticleCollision(
    Particle& a,
    Particle& b,
    const SimulationConfig& config
) {
    Vec2 delta = b.position - a.position;
    float dist = delta.length();
    const float minDist = a.radius + b.radius;

    if (dist >= minDist) {
        return false;
    }

    Vec2 normal;
    if (dist < Epsilon) {
        normal = Vec2{1.0f, 0.0f};
        dist = Epsilon;
    } else {
        normal = delta * (1.0f / dist);
    }

    const float penetration = minDist - dist;
    const Vec2 correction = normal * (penetration * 0.5f);

    a.position -= correction;
    b.position += correction;

    const Vec2 relativeVelocity = b.velocity - a.velocity;
    const float velocityAlongNormal = Vec2::dot(relativeVelocity, normal);

    if (velocityAlongNormal < 0.0f) {
        const float impulseMagnitude =
            -(1.0f + config.restitution) * velocityAlongNormal * 0.5f;

        const Vec2 impulse = normal * impulseMagnitude;

        a.velocity -= impulse;
        b.velocity += impulse;
    }

    return true;
}

} // namespace particles_cpu
