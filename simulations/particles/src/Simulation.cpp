#include "Simulation.hpp"
#include "ParticlePhysics.hpp"
#include "thread_pool.hpp"

#include <simulation/CollisionPairs.hpp>
#include <simulation/ParallelUpdate.hpp>
#include <simulation/SpatialQuery.hpp>
#include <simulation/SimulationUtils.hpp>
#include <simulation/EntityBrush.hpp>

#include <algorithm>
#include <memory>
#include <random/Random.hpp>

namespace particles {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;

Vec2 particlePosition(const Particle& particle) {
    return particle.position;
}

} // namespace

Simulation::Simulation(SimulationConfig config)
    : Base(config),
      m_grid(m_config.gridCellSize),
      m_threadPool(std::make_unique<ThreadPool>(simfw::simulation::hardwareWorkerCount())) {
    m_config.cellSize = m_config.gridCellSize;
    m_config.entityCount = m_config.maxParticleCount;

    m_entities.reserve(m_config.maxParticleCount);
    m_stats.maxParticleCount = m_config.maxParticleCount;
    updateStatsCount();
}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::updateStatsCount() {
    m_stats.particleCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
    m_stats.maxParticleCount = m_config.maxParticleCount;
}

void Simulation::integrateParticleRange(
    std::size_t beginIndex,
    std::size_t endIndex,
    float dt
) {
    const Vec2 gravity{0.0f, m_config.gravity};

    for (std::size_t i = beginIndex; i < endIndex; ++i) {
        Particle& particle = m_entities[i];

        particle.velocity += gravity * dt;
        particle.position += particle.velocity * dt;
        particle.velocity *= m_config.damping;

        solveParticleBounds(particle, m_config);
    }
}

void Simulation::integrateParticles(float dt) {
    const auto backend = m_config.execution.computeBackend;

    if (backend == simfw::simulation::ComputeBackend::CpuScalar) {
        integrateParticleRange(0, m_entities.size(), dt);
        return;
    }

    const bool useParallel =
        backend == simfw::simulation::ComputeBackend::GpuCompute
            ? true
            : m_config.execution.useParallelUpdate;

    simfw::runParallelUpdate<
        ThreadPool,
        ParticleUpdateScratch,
        SimulationStats
    >(
        m_threadPool.get(),
        m_entities.size(),
        MinItemsPerParallelTask,
        useParallel,
        [this, dt](
            std::size_t beginIndex,
            std::size_t endIndex,
            ParticleUpdateScratch&,
            SimulationStats&
        ) {
            integrateParticleRange(beginIndex, endIndex, dt);
        },
        [](const SimulationStats&) {}
    );
}

void Simulation::buildSpatialIndex() {
    simfw::simulation::setupSpatialIndex(
        m_grid,
        m_config.gridCellSize,
        m_config.execution.useSpatialGrid,
        m_entities,
        particlePosition
    );
}

void Simulation::resolveCollisions() {
    const float queryRadius = m_config.particleRadius * 2.0f;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        m_entities.size(),
        [this, queryRadius](std::size_t particleIndex, std::vector<int>& candidates) {
            simfw::simulation::collectCandidates(
                m_grid,
                m_entities[particleIndex].position,
                queryRadius,
                simfw::simulation::makeSpatialQueryOptions<int>(
                    m_config.execution.useSpatialGrid,
                    m_entities.size()
                ),
                candidates
            );
        },
        [this](int firstIndex, int secondIndex) {
            ++m_stats.collisionChecks;

            Particle& first = m_entities[static_cast<std::size_t>(firstIndex)];
            Particle& second = m_entities[static_cast<std::size_t>(secondIndex)];

            if (resolveParticleCollision(first, second, m_config)) {
                ++m_stats.collisionsResolved;

                solveParticleBounds(first, m_config);
                solveParticleBounds(second, m_config);
            }
        }
    );
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.maxParticleCount = m_config.maxParticleCount;

    dt = std::min(dt, 1.0f / 30.0f);

    integrateParticles(dt);
    buildSpatialIndex();
    resolveCollisions();

    for (auto& particle : m_entities) {
        solveParticleBounds(particle, m_config);
    }

    updateStatsCount();
}

void Simulation::spawn(const Vec2& pos) {
    for (int i = 0; i < m_config.spawnCount &&
                    m_entities.size() < m_config.maxParticleCount; ++i) {
        const Vec2 spawnOffset = simfw::simulation::randomDiscOffset(m_config.brushRadius);

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

} // namespace particles
