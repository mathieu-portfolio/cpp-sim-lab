#pragma once

#include "Particle.hpp"
#include "SpatialGrid.hpp"

#include <cstddef>
#include <vector>

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;

    float cellSize = 16.0f;
    bool useSpatialGrid = true;

    std::size_t entityCount = 500;
};

struct SimulationStats {
    std::size_t entityCount = 0;
    std::size_t collisionChecks = 0;
};

class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});
    void update();
    void reset();

    const std::vector<Particle>& getEntities() const { return m_particles; }
    SimulationStats getStats() const { return m_stats; }

private:
    std::vector<Particle> m_particles;
    SimulationConfig m_config;
    SimulationStats m_stats;
    SpatialGrid m_grid;
};
