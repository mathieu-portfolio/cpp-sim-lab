#pragma once

#include "Particle.hpp"
#include "SpatialGrid.hpp"

#include <cstddef>
#include <vector>

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;

    float gravity = 500.0f;
    float damping = 0.999f;
    float bounce = -0.7f;
    float restitution = 0.8f;

    float particleRadius = 4.0f;
    int spawnCount = 8;
    std::size_t maxParticleCount = 1000;

    float gridCellSize = 16.0f;

    // Compatibility with the attempted generic naming.
    // Prefer maxParticleCount / particleCount in particles_cpu-specific code.
    float cellSize = 16.0f;
    bool useSpatialGrid = true;
    std::size_t entityCount = 0;
};

struct SimulationStats {
    std::size_t particleCount = 0;
    std::size_t maxParticleCount = 0;
    std::size_t collisionChecks = 0;
    std::size_t collisionsResolved = 0;

    // Compatibility alias for generic stats naming.
    std::size_t entityCount = 0;
};

class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});

    void update(float dt);
    void reset();
    void spawn(const Vec2& pos);
    void clear();

    const std::vector<Particle>& getParticles() const { return m_particles; }

    // Compatibility alias for generic code.
    const std::vector<Particle>& getEntities() const { return m_particles; }

    SimulationConfig& getConfig() { return m_config; }
    const SimulationConfig& getConfig() const { return m_config; }

    const SpatialGrid& getGrid() const { return m_grid; }

    SimulationStats getStats() const;

private:
    std::vector<Particle> m_particles;
    SimulationConfig m_config;
    SimulationStats m_stats;
    SpatialGrid m_grid;

    void updateStatsCount();
};
