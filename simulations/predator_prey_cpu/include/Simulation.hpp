#pragma once

#include "Boid.hpp"

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <vector>

namespace predator_prey_cpu {

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;
    float gridCellSize = 50.0f;

    float preyPerceptionRadius = 50.0f;
    float preySeparationRadius = 18.0f;
    float predatorPerceptionRadius = 130.0f;
    float catchRadius = 10.0f;

    float preyMaxSpeed = 120.0f;
    float predatorMaxSpeed = 140.0f;
    float preyMaxForce = 85.0f;
    float predatorMaxForce = 95.0f;

    float alignmentWeight = 1.0f;
    float cohesionWeight = 0.4f;
    float separationWeight = 1.8f;
    float fleeWeight = 2.2f;
    float chaseWeight = 1.8f;

    float predatorDrainPerSecond = 0.08f;
    float predatorEnergyGainOnCatch = 0.65f;

    simfw::simulation::SimulationExecutionConfig execution{false, true};

    std::size_t preyCount = 180;
    std::size_t predatorCount = 8;
    std::size_t maxPreyCount = 2000;
    int spawnCount = 8;
    float brushRadius = 24.0f;
    std::size_t entityCount = 188;
};

struct SimulationStats {
    std::size_t preyCount = 0;
    std::size_t predatorCount = 0;
    std::size_t entityCount = 0;
    std::size_t catches = 0;
    std::size_t occupiedGridCells = 0;
};

class Simulation : public simfw::SimulationBase<SimulationConfig, SimulationStats, Boid> {
public:
    using Grid = simfw::SpatialHashGrid<std::size_t>;
    explicit Simulation(SimulationConfig config = {});
    void update(float dt);
    void reset();
    void spawn(const Vec2& position);
    const Grid& getGrid() const { return m_grid; }
    const std::vector<Boid>& getBoids() const { return m_entities; }

private:
    Grid m_grid;
    std::vector<Boid> m_previousBoids;

    void updateStats();
};

} // namespace predator_prey_cpu
