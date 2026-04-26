#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

namespace {
constexpr float Width = 800.0f;
constexpr float Height = 800.0f;
constexpr float Bounce = -0.45f;
constexpr float Damping = 0.992f;
constexpr float Restitution = 0.15f;
constexpr float Epsilon = 0.0001f;

void solveBounds(Particle& p) {
    if (p.position.x - p.radius < 0.0f) {
        p.position.x = p.radius;
        if (p.velocity.x < 0.0f) {
            p.velocity.x *= Bounce;
        }
    }

    if (p.position.x + p.radius > Width) {
        p.position.x = Width - p.radius;
        if (p.velocity.x > 0.0f) {
            p.velocity.x *= Bounce;
        }
    }

    if (p.position.y + p.radius > Height) {
        p.position.y = Height - p.radius;
        if (p.velocity.y > 0.0f) {
            p.velocity.y *= Bounce;
        }
    }

    if (p.position.y - p.radius < 0.0f) {
        p.position.y = p.radius;
        if (p.velocity.y < 0.0f) {
            p.velocity.y *= Bounce;
        }
    }
}

void resolveCollisions(std::vector<Particle>& particles, SimulationStats& stats) {
    const std::size_t count = particles.size();

    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = i + 1; j < count; ++j) {
            ++stats.collisionChecks;

            Particle& a = particles[i];
            Particle& b = particles[j];

            Vec2 delta = b.position - a.position;
            float dist = delta.length();

            const float minDist = a.radius + b.radius;

            if (dist >= minDist) {
                continue;
            }

            ++stats.collisionsResolved;

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
                    -(1.0f + Restitution) * velocityAlongNormal * 0.5f;

                const Vec2 impulse = normal * impulseMagnitude;

                a.velocity -= impulse;
                b.velocity += impulse;
            }

            solveBounds(a);
            solveBounds(b);
        }
    }
}
}

Simulation::Simulation(std::size_t maxParticleCount)
    : m_maxParticleCount(maxParticleCount) {
    m_particles.reserve(m_maxParticleCount);
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.maxParticleCount = m_maxParticleCount;

    dt = std::min(dt, 1.0f / 30.0f);

    const Vec2 gravity{0.0f, 200.0f};

    for (auto& p : m_particles) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;
        p.velocity *= Damping;

        solveBounds(p);
    }

    resolveCollisions(m_particles, m_stats);

    for (auto& p : m_particles) {
        solveBounds(p);
    }

    constexpr float sleepVelocityThreshold = 5.0f;

    m_particles.erase(
        std::remove_if(
            m_particles.begin(),
            m_particles.end(),
            [](const Particle& p) {
                return std::abs(p.velocity.y) < sleepVelocityThreshold &&
                       p.position.y + p.radius >= Height - 1.0f;
            }
        ),
        m_particles.end()
    );

    m_stats.particleCount = m_particles.size();
}

void Simulation::spawn(const Vec2& pos) {
    constexpr int spawnCount = 2;

    for (int i = 0; i < spawnCount && m_particles.size() < m_maxParticleCount; ++i) {
        const Vec2 spawnOffset{
            Random::range(-20.0f, 20.0f),
            Random::range(-20.0f, 20.0f)
        };

        m_particles.push_back({
            pos + spawnOffset,
            Vec2{
                Random::range(-80.0f, 80.0f),
                Random::range(-120.0f, -40.0f)
            },
            4.0f
        });
    }
}

void Simulation::clear() {
    m_particles.clear();
    m_stats = {};
    m_stats.maxParticleCount = m_maxParticleCount;
}

void Simulation::reset() {
    clear();

    constexpr int initialParticleCount = 50;

    for (int i = 0; i < initialParticleCount && m_particles.size() < m_maxParticleCount; ++i) {
        m_particles.push_back({
            Vec2{
                Random::range(100.0f, 700.0f),
                Random::range(100.0f, 300.0f)
            },
            Vec2{
                Random::range(-60.0f, 60.0f),
                Random::range(-100.0f, 20.0f)
            },
            4.0f
        });
    }

    m_stats.particleCount = m_particles.size();
}

SimulationStats Simulation::getStats() const {
    return m_stats;
}
