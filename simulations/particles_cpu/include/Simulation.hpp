#pragma once

#include "Particle.hpp"

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <memory>
#include <vector>

class ThreadPool;

namespace particles_cpu {

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

    float cellSize = 16.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
    std::size_t entityCount = 0;
};

struct SimulationStats {
    std::size_t particleCount = 0;
    std::size_t maxParticleCount = 0;
    std::size_t collisionChecks = 0;
    std::size_t collisionsResolved = 0;
    std::size_t entityCount = 0;
};

class Simulation
    : public simfw::SimulationBase<SimulationConfig, SimulationStats, Particle> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Particle>;
    using Grid = simfw::SpatialHashGrid<int>;

    explicit Simulation(SimulationConfig config = {});
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    void update(float dt);
    void reset();
    void spawn(const Vec2& pos);
    void clear();

    const std::vector<Particle>& getParticles() const { return m_entities; }

    const Grid& getGrid() const { return m_grid; }

private:
    struct ParticleUpdateScratch {};

    Grid m_grid;
    std::unique_ptr<ThreadPool> m_threadPool;

    void updateStatsCount();
    void integrateParticles(float dt);
    void integrateParticleRange(
        std::size_t beginIndex,
        std::size_t endIndex,
        float dt
    );
    void buildSpatialIndex();
    void resolveCollisions();
};

} // namespace particles_cpu
