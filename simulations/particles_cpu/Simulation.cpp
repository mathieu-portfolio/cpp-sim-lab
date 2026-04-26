#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

namespace {
constexpr float Epsilon = 0.0001f;

void solveBounds(Particle& p, const SimulationConfig& config) {
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

void resolveCollisionsGrid(
    std::vector<Particle>& particles,
    SpatialGrid& grid,
    SimulationStats& stats,
    const SimulationConfig& config
) {
    std::vector<int> candidates;

    for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
        Particle& a = particles[i];

        candidates.clear();
        grid.queryNeighbors(a.position, candidates);

        for (int j : candidates) {
            if (j <= i) {
                continue;
            }

            ++stats.collisionChecks;

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
                    -(1.0f + config.restitution) * velocityAlongNormal * 0.5f;

                const Vec2 impulse = normal * impulseMagnitude;

                a.velocity -= impulse;
                b.velocity += impulse;
            }

            solveBounds(a, config);
            solveBounds(b, config);
        }
    }
}
}

Simulation::Simulation(SimulationConfig config)
    : m_config(config),
      m_grid(config.gridCellSize) {
    m_particles.reserve(m_config.maxParticleCount);
    m_stats.maxParticleCount = m_config.maxParticleCount;
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.maxParticleCount = m_config.maxParticleCount;

    dt = std::min(dt, 1.0f / 30.0f);

    const Vec2 gravity{0.0f, m_config.gravity};

    for (auto& p : m_particles) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;
        p.velocity *= m_config.damping;

        solveBounds(p, m_config);
    }

    m_grid.build(m_particles);
    resolveCollisionsGrid(m_particles, m_grid, m_stats, m_config);

    for (auto& p : m_particles) {
        solveBounds(p, m_config);
    }

    m_stats.particleCount = m_particles.size();
}

void Simulation::spawn(const Vec2& pos) {
    for (int i = 0; i < m_config.spawnCount &&
                    m_particles.size() < m_config.maxParticleCount; ++i) {
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
            m_config.particleRadius
        });
    }
}

void Simulation::clear() {
    m_particles.clear();

    m_stats = {};
    m_stats.maxParticleCount = m_config.maxParticleCount;
}

void Simulation::reset() {
    clear();

    constexpr int initialParticleCount = 50;

    for (int i = 0; i < initialParticleCount &&
                    m_particles.size() < m_config.maxParticleCount; ++i) {
        m_particles.push_back({
            Vec2{
                Random::range(100.0f, m_config.width - 100.0f),
                Random::range(100.0f, 300.0f)
            },
            Vec2{
                Random::range(-60.0f, 60.0f),
                Random::range(-100.0f, 20.0f)
            },
            m_config.particleRadius
        });
    }

    m_stats.particleCount = m_particles.size();
}

SimulationStats Simulation::getStats() const {
    return m_stats;
}
