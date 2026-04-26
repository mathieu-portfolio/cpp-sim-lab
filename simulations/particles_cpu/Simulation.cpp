#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

Simulation::Simulation(std::size_t maxParticleCount)
    : m_maxParticleCount(maxParticleCount) {
    m_particles.reserve(m_maxParticleCount);
}

static void resolveCollisions(std::vector<Particle>& particles) {
    constexpr float restitution = 0.35f;
    constexpr float epsilon = 0.0001f;

    const std::size_t count = particles.size();

    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            Particle& a = particles[i];
            Particle& b = particles[j];

            Vec2 delta = b.position - a.position;
            float dist = delta.length();

            const float minDist = a.radius + b.radius;

            if (dist >= minDist) {
                continue;
            }

            Vec2 normal;

            if (dist < epsilon) {
                normal = Vec2{1.0f, 0.0f};
                dist = minDist;
            } else {
                normal = delta * (1.0f / dist);
            }

            const float penetration = minDist - dist;
            const Vec2 correction = normal * (penetration * 0.5f);

            a.position -= correction;
            b.position += correction;

            const Vec2 relativeVelocity = b.velocity - a.velocity;
            const float velocityAlongNormal = Vec2::dot(relativeVelocity, normal);

            if (velocityAlongNormal > 0.0f) {
                continue;
            }

            const float impulseMagnitude = -(1.0f + restitution) * velocityAlongNormal * 0.5f;
            const Vec2 impulse = normal * impulseMagnitude;

            a.velocity -= impulse;
            b.velocity += impulse;
        }
    }
}

void Simulation::update(float dt) {
    constexpr float width = 800.0f;
    constexpr float height = 800.0f;
    constexpr float bounce = -0.65f;
    constexpr float damping = 0.995f;

    const Vec2 gravity{0.0f, 200.0f};

    for (auto& p : m_particles) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;

        if (p.position.y + p.radius > height) {
            p.position.y = height - p.radius;
            p.velocity.y *= bounce;
        }

        if (p.position.x - p.radius < 0.0f) {
            p.position.x = p.radius;
            p.velocity.x *= bounce;
        }

        if (p.position.x + p.radius > width) {
            p.position.x = width - p.radius;
            p.velocity.x *= bounce;
        }

        p.velocity *= damping;
    }

    resolveCollisions(m_particles);

    constexpr float sleepVelocityThreshold = 5.0f;

    m_particles.erase(
        std::remove_if(
            m_particles.begin(),
            m_particles.end(),
            [](const Particle& p) {
                return std::abs(p.velocity.y) < sleepVelocityThreshold &&
                       p.position.y >= 796.0f;
            }
        ),
        m_particles.end()
    );
}

void Simulation::spawn(const Vec2& pos) {
    constexpr int spawnCount = 5;

    for (int i = 0; i < spawnCount && m_particles.size() < m_maxParticleCount; ++i) {
        const Vec2 spawnOffset{
            Random::range(-12.0f, 12.0f),
            Random::range(-12.0f, 12.0f)
        };

        m_particles.push_back({
            pos + spawnOffset,
            Vec2{
                Random::range(-100.0f, 100.0f),
                Random::range(-150.0f, -50.0f)
            },
            4.0f
        });
    }
}

void Simulation::clear() {
    m_particles.clear();
}

void Simulation::reset() {
    clear();

    constexpr int initialParticleCount = 100;

    for (int i = 0; i < initialParticleCount && m_particles.size() < m_maxParticleCount; ++i) {
        m_particles.push_back({
            Vec2{
                Random::range(100.0f, 700.0f),
                Random::range(100.0f, 300.0f)
            },
            Vec2{
                Random::range(-80.0f, 80.0f),
                Random::range(-120.0f, 20.0f)
            },
            4.0f
        });
    }
}

SimulationStats Simulation::getStats() const {
    return SimulationStats{
        m_particles.size(),
        m_maxParticleCount
    };
}
