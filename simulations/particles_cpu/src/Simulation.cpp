#include "Simulation.hpp"
#include "ParticlePhysics.hpp"

#include <algorithm>
#include <random/Random.hpp>

namespace {
Vec2 particlePosition(const Particle& particle) {
    return particle.position;
}

void resolveCollisionsGrid(
    std::vector<Particle>& particles,
    Simulation::Grid& grid,
    SimulationStats& stats,
    const SimulationConfig& config
) {
    std::vector<int> candidates;

    for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
        Particle& a = particles[i];

        candidates.clear();
        grid.queryCellsAround(a.position, 1, candidates);

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
    : Base(config),
      m_grid(m_config.gridCellSize) {
    m_config.cellSize = m_config.gridCellSize;
    m_config.entityCount = m_config.maxParticleCount;

    m_entities.reserve(m_config.maxParticleCount);
    m_stats.maxParticleCount = m_config.maxParticleCount;
    updateStatsCount();
}

void Simulation::updateStatsCount() {
    m_stats.particleCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
    m_stats.maxParticleCount = m_config.maxParticleCount;
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.maxParticleCount = m_config.maxParticleCount;

    dt = std::min(dt, 1.0f / 30.0f);

    const Vec2 gravity{0.0f, m_config.gravity};

    for (auto& p : m_entities) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;
        p.velocity *= m_config.damping;

        solveParticleBounds(p, m_config);
    }

    if (m_config.useSpatialGrid) {
        m_grid.setCellSize(m_config.gridCellSize);
        m_grid.build(m_entities, particlePosition);
        resolveCollisionsGrid(m_entities, m_grid, m_stats, m_config);
    } else {
        m_grid.clear();
    }

    for (auto& p : m_entities) {
        solveParticleBounds(p, m_config);
    }

    updateStatsCount();
}

void Simulation::spawn(const Vec2& pos) {
    for (int i = 0; i < m_config.spawnCount &&
                    m_entities.size() < m_config.maxParticleCount; ++i) {
        const Vec2 spawnOffset{
            Random::range(-20.0f, 20.0f),
            Random::range(-20.0f, 20.0f)
        };

        m_entities.push_back({
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
    m_entities.clear();
    m_grid.clear();

    m_stats = {};
    m_stats.maxParticleCount = m_config.maxParticleCount;
    updateStatsCount();
}

void Simulation::reset() {
    clear();

    constexpr int initialParticleCount = 50;

    for (int i = 0; i < initialParticleCount &&
                    m_entities.size() < m_config.maxParticleCount; ++i) {
        m_entities.push_back({
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
