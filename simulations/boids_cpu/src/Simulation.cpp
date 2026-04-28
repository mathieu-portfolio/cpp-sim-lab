#include "Simulation.hpp"
#include "BoidBehavior.hpp"

#include "thread_pool.hpp"

#include <simulation/ParallelUpdate.hpp>
#include <simulation/SpatialQuery.hpp>

#include <algorithm>
#include <memory>
#include <random/Random.hpp>
#include <thread>

namespace boids_cpu {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;

void addNeighborsFromCandidateList(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& candidates,
    float perceptionRadius,
    float separationRadius,
    std::vector<std::size_t>& perceptionNeighbors,
    std::vector<std::size_t>& separationNeighbors,
    SimulationStats& stats
) {
    perceptionNeighbors.clear();
    separationNeighbors.clear();

    for (std::size_t candidateIndex : candidates) {
        ++stats.neighborChecks;

        const float distance =
            (boids[candidateIndex].position - boids[boidIndex].position).length();

        if (distance < perceptionRadius) {
            perceptionNeighbors.push_back(candidateIndex);
        }

        if (distance < separationRadius) {
            separationNeighbors.push_back(candidateIndex);
        }
    }
}

Vec2 boidPosition(const Boid& boid) {
    return boid.position;
}

std::size_t hardwareWorkerCount() {
    const unsigned int hardwareThreads = std::thread::hardware_concurrency();

    if (hardwareThreads == 0) {
        return 1;
    }

    return static_cast<std::size_t>(hardwareThreads);
}
}

Simulation::Simulation(SimulationConfig config)
    : Base(config),
      m_grid(m_config.gridCellSize),
      m_threadPool(std::make_unique<ThreadPool>(hardwareWorkerCount())) {
    normalizeConfigCounts();
    reset();
}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::normalizeConfigCounts() {
    constexpr std::size_t defaultCount = SimulationConfig::DefaultBoidCount;

    if (m_config.boidCount == defaultCount && m_config.entityCount != defaultCount) {
        m_config.boidCount = m_config.entityCount;
    } else {
        m_config.entityCount = m_config.boidCount;
    }
}

void Simulation::updateStatsCount() {
    m_stats.boidCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
}

void Simulation::reset() {
    normalizeConfigCounts();

    m_entities.clear();
    m_entities.reserve(m_config.boidCount);

    m_previousBoids.clear();
    m_previousBoids.reserve(m_config.boidCount);

    for (std::size_t i = 0; i < m_config.boidCount; ++i) {
        m_entities.push_back({
            Vec2{
                Random::range(0.0f, m_config.width),
                Random::range(0.0f, m_config.height)
            },
            Vec2{
                Random::range(-50.0f, 50.0f),
                Random::range(-50.0f, 50.0f)
            }
        });
    }

    m_stats = {};
    updateStatsCount();
}

void Simulation::beginFrame() {
    normalizeConfigCounts();

    m_stats = {};
    updateStatsCount();
}

void Simulation::snapshotBoids() {
    m_previousBoids = m_entities;
}

void Simulation::buildSpatialIndex() {
    m_grid.setCellSize(m_config.gridCellSize);

    if (m_config.useSpatialGrid) {
        m_grid.build(m_previousBoids, boidPosition);
        m_stats.occupiedGridCells = m_grid.getCells().size();
    } else {
        m_grid.clear();
    }
}

void Simulation::collectCandidateBoids(
    std::size_t boidIndex,
    float queryRadius,
    BoidUpdateScratch& scratch,
    SimulationStats& stats
) {
    simfw::simulation::collectCandidates(
        m_grid,
        m_previousBoids[boidIndex].position,
        queryRadius,
        simfw::simulation::makeSpatialQueryOptionsExcluding(
            m_config.useSpatialGrid,
            m_previousBoids.size(),
            boidIndex
        ),
        scratch.candidates
    );

    stats.neighborCandidates += scratch.candidates.size();
}

void Simulation::updateBoidRange(
    std::size_t beginIndex,
    std::size_t endIndex,
    float dt,
    float queryRadius,
    BoidUpdateScratch& scratch,
    SimulationStats& stats
) {
    for (std::size_t i = beginIndex; i < endIndex; ++i) {
        collectCandidateBoids(i, queryRadius, scratch, stats);

        addNeighborsFromCandidateList(
            i,
            m_previousBoids,
            scratch.candidates,
            m_config.perceptionRadius,
            m_config.separationRadius,
            scratch.perceptionNeighbors,
            scratch.separationNeighbors,
            stats
        );

        Vec2 align = computeAlignment(
            i,
            m_previousBoids,
            scratch.perceptionNeighbors,
            m_config.maxSpeed
        );

        Vec2 coh = computeCohesion(
            i,
            m_previousBoids,
            scratch.perceptionNeighbors,
            m_config.maxSpeed
        );

        Vec2 sep = computeSeparation(
            i,
            m_previousBoids,
            scratch.separationNeighbors,
            m_config.maxSpeed
        );

        Vec2 force =
            align * m_config.alignmentWeight +
            coh * m_config.cohesionWeight +
            sep * m_config.separationWeight;

        force = limitLength(force, m_config.maxForce);

        Boid& boid = m_entities[i];
        const Boid& previousBoid = m_previousBoids[i];

        boid.velocity = previousBoid.velocity + force * dt;
        boid.velocity = limitLength(boid.velocity, m_config.maxSpeed);
        boid.position = previousBoid.position + boid.velocity * dt;

        boid.position = wrapPosition(
            boid.position,
            m_config.width,
            m_config.height
        );
    }
}

void Simulation::mergeWorkerStats(const SimulationStats& workerStats) {
    m_stats.neighborChecks += workerStats.neighborChecks;
    m_stats.neighborCandidates += workerStats.neighborCandidates;
}

void Simulation::updateBoids(float dt) {
    const float queryRadius =
        std::max(m_config.perceptionRadius, m_config.separationRadius);

    simfw::runParallelUpdate<ThreadPool, BoidUpdateScratch, SimulationStats>(
        m_threadPool.get(),
        m_entities.size(),
        MinItemsPerParallelTask,
        m_config.useParallelUpdate,
        [this, dt, queryRadius](
            std::size_t beginIndex,
            std::size_t endIndex,
            BoidUpdateScratch& scratch,
            SimulationStats& stats
        ) {
            updateBoidRange(
                beginIndex,
                endIndex,
                dt,
                queryRadius,
                scratch,
                stats
            );
        },
        [this](const SimulationStats& stats) {
            mergeWorkerStats(stats);
        }
    );
}

void Simulation::update(float dt) {
    beginFrame();
    snapshotBoids();
    buildSpatialIndex();
    updateBoids(dt);
}

} // namespace boids_cpu
