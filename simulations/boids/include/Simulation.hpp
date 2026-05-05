#pragma once

#include "Boid.hpp"
#include "BoidBehavior.hpp"

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>
#include <simulation/NeighborScratch.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

class ThreadPool;

namespace boids {

struct SimulationConfig {
    static constexpr std::size_t DefaultBoidCount = 200;

    float width = 800.0f;
    float height = 800.0f;

    float perceptionRadius = 50.0f;
    float separationRadius = 20.0f;

    float maxSpeed = 120.0f;
    float maxForce = 80.0f;

    float alignmentWeight = 1.0f;
    float cohesionWeight = 0.4f;
    float separationWeight = 2.0f;
    float wanderWeight = 0.0f;
    float wanderJitter = 0.65f;

    simfw::simulation::SimulationExecutionConfig execution{false, true};
    float gridCellSize = 50.0f;

    std::size_t boidCount = DefaultBoidCount;
    std::size_t maxBoidCount = 2000;
    int spawnCount = 8;
    float brushRadius = 24.0f;
    std::size_t entityCount = DefaultBoidCount;
};

struct SimulationStats {
    std::size_t boidCount = 0;
    std::size_t entityCount = 0;

    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
};

class Simulation
    : public simfw::SimulationBase<SimulationConfig, SimulationStats, Boid> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Boid>;
    using Grid = simfw::SpatialHashGrid<std::size_t>;

    explicit Simulation(SimulationConfig config = {});
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    void update(float dt);
    void reset();
    void spawn(const Vec2& position);

    const std::vector<Boid>& getBoids() const { return m_entities; }

    const Grid& getGrid() const { return m_grid; }

    std::span<const WeightedBoidBehavior> getBehaviors() const { return m_behaviors; }
    void setBehaviors(std::span<const WeightedBoidBehavior> behaviors);
    void resetBehaviors();

private:
    using BoidUpdateScratch = simfw::simulation::NeighborScratch<std::size_t>;

    Grid m_grid;
    std::vector<Boid> m_previousBoids;
    std::vector<WeightedBoidBehavior> m_behaviors;
    std::unique_ptr<ThreadPool> m_threadPool;

    void normalizeConfigCounts();
    void updateStatsCount();

    void beginFrame();
    void snapshotBoids();
    void buildSpatialIndex();

    void updateBoids(float dt);
    void updateBoidRange(
        std::size_t beginIndex,
        std::size_t endIndex,
        float dt,
        float queryRadius,
        BoidUpdateScratch& scratch,
        SimulationStats& stats
    );

    void collectCandidateBoids(
        std::size_t boidIndex,
        float queryRadius,
        BoidUpdateScratch& scratch,
        SimulationStats& stats
    );

    void mergeWorkerStats(const SimulationStats& workerStats);
};

} // namespace boids
