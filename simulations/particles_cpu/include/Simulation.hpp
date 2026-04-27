#pragma once

#include "Particle.hpp"
#include "SpatialGrid.hpp"

#include <cstddef>
#include <vector>

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;
    float gravity = 200.0f;
    float bounce = -0.45f;
    float damping = 0.992f;
    float restitution = 0.15f;
    float particleRadius = 4.0f;
    int spawnCount = 2;
    std::size_t maxParticleCount = 1000;
    float gridCellSize = 16.0f;
};

struct SimulationStats {
    std::size_t particleCount = 0;
    std::size_t maxParticleCount = 0;
    std::size_t collisionChecks = 0;
    std::size_t collisionsResolved = 0;
};

class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});

    void update(float dt);
    void spawn(const Vec2& pos);
    void clear();
    void reset();

    const std::vector<Particle>& getParticles() const { return m_particles; }
    SimulationStats getStats() const;
    const SimulationConfig& getConfig() const { return m_config; }
    const SpatialGrid& getGrid() const { return m_grid; }

private:
    std::vector<Particle> m_particles;
    SimulationConfig m_config;
    SimulationStats m_stats;

    SpatialGrid m_grid;
};
