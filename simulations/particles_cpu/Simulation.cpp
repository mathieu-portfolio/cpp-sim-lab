#include "Simulation.hpp"

#include <random/Random.hpp>
#include <cmath>

Simulation::Simulation(std::size_t maxParticleCount)
    : m_maxParticleCount(maxParticleCount) {
    m_particles.reserve(m_maxParticleCount);
}

static void resolveCollisions(std::vector<Particle>& particles) {
    const float restitution = 0.8f;

    const std::size_t count = particles.size();

    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            Particle& a = particles[i];
            Particle& b = particles[j];

            Vec2 delta = b.position - a.position;
            float dist = delta.length();

            float minDist = a.radius + b.radius;

            if (dist == 0.0f || dist >= minDist)
                continue;

            Vec2 normal = delta * (1.0f / dist);

            float penetration = minDist - dist;

            // position correction
            Vec2 correction = normal * (penetration * 0.5f);
            a.position -= correction;
            b.position += correction;

            // relative velocity
            Vec2 relVel = b.velocity - a.velocity;
            float velAlongNormal = Vec2::dot(relVel, normal);

            if (velAlongNormal > 0.0f)
                continue;

            float impulseMag = -(1.0f + restitution) * velAlongNormal * 0.5f;

            Vec2 impulse = normal * impulseMag;

            a.velocity -= impulse;
            b.velocity += impulse;
        }
    }
}

void Simulation::update(float dt) {
    const Vec2 gravity{0.0f, 200.0f};

    for (auto& p : m_particles) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;

        if (p.position.y > 800.0f) {
            p.position.y = 800.0f;
            p.velocity.y *= -0.8f;
        }

        if (p.position.x < 0.0f) {
            p.position.x = 0.0f;
            p.velocity.x *= -0.8f;
        }

        if (p.position.x > 800.0f) {
            p.position.x = 800.0f;
            p.velocity.x *= -0.8f;
        }

        p.velocity *= 0.999f;
    }

    // naive O(n^2) collisions
    resolveCollisions(m_particles);

    // lifecycle
    constexpr float sleepVelocityThreshold = 5.0f;

    m_particles.erase(
        std::remove_if(
            m_particles.begin(),
            m_particles.end(),
            [](const Particle& p) {
                return std::abs(p.velocity.y) < sleepVelocityThreshold &&
                       p.position.y >= 799.0f;
            }
        ),
        m_particles.end()
    );
}

void Simulation::spawn(const Vec2& pos) {
    constexpr int spawnCount = 10;

    for (int i = 0; i < spawnCount && m_particles.size() < m_maxParticleCount; ++i) {
        m_particles.push_back({
            pos,
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
