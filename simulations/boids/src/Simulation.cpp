#include "Simulation.hpp"
#include "BoidBehavior.hpp"

#include "thread_pool.hpp"

#include <simulation/ParallelUpdate.hpp>
#include <simulation/SpatialQuery.hpp>
#include <simulation/StatsReduction.hpp>
#include <simulation/SimulationUtils.hpp>
#include <simulation/EntityBrush.hpp>

#include <algorithm>
#include <memory>
#include <random/Random.hpp>

namespace boids {
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

} // namespace

Simulation::Simulation(SimulationConfig config)
    : Base(config),
      m_grid(m_config.gridCellSize),
      m_behaviors(
          defaultBoidBehaviors().begin(),
          defaultBoidBehaviors().end()
      ),
      m_threadPool(std::make_unique<ThreadPool>(simfw::simulation::hardwareWorkerCount())) {
    normalizeConfigCounts();
    reset();
}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::setBehaviors(std::span<const WeightedBoidBehavior> behaviors) {
    m_behaviors.assign(behaviors.begin(), behaviors.end());
}

void Simulation::resetBehaviors() {
    setBehaviors(defaultBoidBehaviors());
}

void Simulation::normalizeConfigCounts() {
    simfw::simulation::syncEntityCount(
        SimulationConfig::DefaultBoidCount,
        m_config.boidCount,
        m_config
    );
}

void Simulation::updateStatsCount() {
    m_stats.boidCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
    m_config.entityCount = m_entities.size();
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


void Simulation::spawn(const Vec2& position) {
    simfw::simulation::spawnBrush(
        m_entities.size(),
        m_config.maxBoidCount,
        m_config.spawnCount,
        [this, position]() {
            m_entities.push_back({
                position + simfw::simulation::randomDiscOffset(m_config.brushRadius),
                Vec2{Random::range(-50.0f, 50.0f), Random::range(-50.0f, 50.0f)}
            });
        }
    );
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
    simfw::simulation::setupSpatialIndex(
        m_grid,
        m_config.gridCellSize,
        m_config.execution.useSpatialGrid,
        m_previousBoids,
        boidPosition
    );

    if (m_config.execution.useSpatialGrid) {
        m_stats.occupiedGridCells = m_grid.getCells().size();
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
            m_config.execution.useSpatialGrid,
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
            scratch.neighbors,
            scratch.secondaryNeighbors,
            stats
        );

        BoidBehaviorContext behaviorContext{
            m_config,
            m_previousBoids,
            BoidCandidateLists{
                scratch.neighbors,
                scratch.secondaryNeighbors
            },
            stats
        };

        const Vec2 acceleration = computeAcceleration(
            i,
            behaviorContext,
            m_behaviors
        );

        Boid& boid = m_entities[i];
        const Boid& previousBoid = m_previousBoids[i];

        boid.velocity = previousBoid.velocity + acceleration * dt;
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
    simfw::sumStatsMembers(
        m_stats,
        workerStats,
        &SimulationStats::neighborChecks,
        &SimulationStats::neighborCandidates
    );
}

void Simulation::updateBoids(float dt) {
    const float queryRadius =
        std::max(m_config.perceptionRadius, m_config.separationRadius);

    simfw::runParallelUpdate<ThreadPool, BoidUpdateScratch, SimulationStats>(
        m_threadPool.get(),
        m_entities.size(),
        MinItemsPerParallelTask,
        m_config.execution.useParallelUpdate,
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

} // namespace boids
