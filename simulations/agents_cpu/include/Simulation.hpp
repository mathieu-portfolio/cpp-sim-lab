#pragma once

#include "Agent.hpp"

#include <simulation/SimulationBase.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <vector>

struct SimulationConfig {
    static constexpr std::size_t DefaultAgentCount = 250;

    float width = 800.0f;
    float height = 800.0f;

    std::size_t agentCount = DefaultAgentCount;
    std::size_t entityCount = DefaultAgentCount;

    float agentRadius = 4.0f;

    float maxSpeed = 140.0f;
    float maxForce = 180.0f;

    float seekWeight = 1.0f;
    float separationWeight = 2.2f;

    float arrivalRadius = 80.0f;
    float targetRadius = 12.0f;
    float separationRadius = 18.0f;

    bool useSpatialGrid = true;
    float gridCellSize = 24.0f;
};

struct SimulationStats {
    std::size_t agentCount = 0;
    std::size_t entityCount = 0;

    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t arrivedCount = 0;
};

class Simulation
    : public simfw::SimulationBase<SimulationConfig, SimulationStats, Agent> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Agent>;
    using Grid = simfw::SpatialHashGrid<std::size_t>;

    explicit Simulation(SimulationConfig config = {});

    void update(float dt);
    void reset();

    const std::vector<Agent>& getAgents() const { return m_entities; }

    const Grid& getGrid() const { return m_grid; }

    void setTarget(Vec2 target);
    Vec2 getTarget() const { return m_target; }

private:
    Grid m_grid;
    Vec2 m_target;

    void normalizeConfigCounts();
    void updateStatsCount();
    Vec2 randomPoint() const;
};
