#pragma once

#include "Boid.hpp"

#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <vector>

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

    bool useSpatialGrid = false;
    float gridCellSize = 50.0f;

    std::size_t boidCount = DefaultBoidCount;
    std::size_t entityCount = DefaultBoidCount;
};

struct SimulationStats {
    std::size_t boidCount = 0;
    std::size_t entityCount = 0;

    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
};

class Simulation {
public:
    using Grid = simfw::SpatialHashGrid<std::size_t>;

    explicit Simulation(SimulationConfig config = {});
    void update(float dt);
    void reset();

    const std::vector<Boid>& getBoids() const { return m_boids; }
    const std::vector<Boid>& getEntities() const { return m_boids; }

    SimulationConfig& getConfig() { return m_config; }
    const SimulationConfig& getConfig() const { return m_config; }

    SimulationStats getStats() const { return m_stats; }

    const Grid& getGrid() const { return m_grid; }

private:
    std::vector<Boid> m_boids;
    SimulationConfig m_config;
    SimulationStats m_stats;
    Grid m_grid;

    void normalizeConfigCounts();
    void updateStatsCount();
};
