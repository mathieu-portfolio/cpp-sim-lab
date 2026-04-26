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
        if (p.velocity.x < 0.0f) p.velocity.x *= Bounce;
    }
    if (p.position.x + p.radius > Width) {
        p.position.x = Width - p.radius;
        if (p.velocity.x > 0.0f) p.velocity.x *= Bounce;
    }
    if (p.position.y + p.radius > Height) {
        p.position.y = Height - p.radius;
        if (p.velocity.y > 0.0f) p.velocity.y *= Bounce;
    }
}

void resolveCollisionsGrid(std::vector<Particle>& particles,
                           SpatialGrid& grid,
                           SimulationStats& stats) {
    std::vector<int> candidates;

    for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
        Particle& a = particles[i];

        candidates.clear();
        grid.queryNeighbors(a.position, candidates);

        for (int j : candidates) {
            if (j <= i) continue;

            ++stats.collisionChecks;

            Particle& b = particles[j];

            Vec2 delta = b.position - a.position;
            float dist = delta.length();
            float minDist = a.radius + b.radius;

            if (dist >= minDist) continue;

            ++stats.collisionsResolved;

            Vec2 normal;
            if (dist < Epsilon) {
                normal = Vec2{1.0f, 0.0f};
                dist = Epsilon;
            } else {
                normal = delta * (1.0f / dist);
            }

            float penetration = minDist - dist;
            Vec2 correction = normal * (penetration * 0.5f);

            a.position -= correction;
            b.position += correction;

            Vec2 relVel = b.velocity - a.velocity;
            float velAlongNormal = Vec2::dot(relVel, normal);

            if (velAlongNormal < 0.0f) {
                float impulseMag =
                    -(1.0f + Restitution) * velAlongNormal * 0.5f;

                Vec2 impulse = normal * impulseMag;
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
    : m_maxParticleCount(maxParticleCount),
      m_grid(16.0f) {
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

    m_grid.build(m_particles);

    resolveCollisionsGrid(m_particles, m_grid, m_stats);

    m_stats.particleCount = m_particles.size();
}

void Simulation::spawn(const Vec2& pos) {
    for (int i = 0; i < 2 && m_particles.size() < m_maxParticleCount; ++i) {
        m_particles.push_back({
            pos,
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
}

void Simulation::reset() {
    clear();
}

SimulationStats Simulation::getStats() const {
    return m_stats;
}
