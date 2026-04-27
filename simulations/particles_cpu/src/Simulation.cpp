#include "Simulation.hpp"
#include "ParticlePhysics.hpp"

#include <algorithm>
#include <random/Random.hpp>

namespace {
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

            if (resolveParticleCollision(a, b, config)) {
                ++stats.collisionsResolved;

                solveParticleBounds(a, config);
                solveParticleBounds(b, config);
            }
        }
    }
}
}

Simulation::Simulation(SimulationConfig config)
    : m_config(config),
      m_grid(config.gridCellSize) {
    m_config.cellSize = m_config.gridCellSize;
    m_config.entityCount = m_config.maxParticleCount;

    m_particles.reserve(m_config.maxParticleCount);
    m_stats.maxParticleCount = m_config.maxParticleCount;
    updateStatsCount();
}

void Simulation::updateStatsCount() {
    m_stats.particleCount = m_particles.size();
    m_stats.entityCount = m_particles.size();
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

        solveParticleBounds(p, m_config);
    }

    if (m_config.useSpatialGrid) {
        m_grid.build(m_particles);
        resolveCollisionsGrid(m_particles, m_grid, m_stats, m_config);
    }

    for (auto& p : m_particles) {
        solveParticleBounds(p, m_config);
    }

    updateStatsCount();
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

    updateStatsCount();
}

void Simulation::clear() {
    m_particles.clear();

    m_stats = {};
    m_stats.maxParticleCount = m_config.maxParticleCount;
    updateStatsCount();
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

    updateStatsCount();
}

SimulationStats Simulation::getStats() const {
    return m_stats;
}
