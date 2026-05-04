#pragma once

#include "Agent.hpp"

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>
#include <spatial/SpatialHashGrid.hpp>

namespace epidemic_cpu {

struct SimulationConfig {
    static constexpr std::size_t DefaultAgentCount = 500;
    float width = 800.0f;
    float height = 800.0f;

    std::size_t agentCount = DefaultAgentCount;
    std::size_t maxAgentCount = 10000;
    int spawnCount = 12;
    float brushRadius = 20.0f;
    std::size_t entityCount = DefaultAgentCount;

    float agentRadius = 3.0f;
    float maxSpeed = 35.0f;
    float steeringJitter = 20.0f;

    float infectionRadius = 16.0f;
    float infectionRate = 0.6f;
    float recoveryTime = 10.0f;
    float initialInfectedRatio = 0.02f;

    float gridCellSize = 20.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
};

struct SimulationStats {
    std::size_t agentCount = 0;
    std::size_t entityCount = 0;
    std::size_t susceptibleCount = 0;
    std::size_t infectedCount = 0;
    std::size_t recoveredCount = 0;

    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;

    float infectedFraction = 0.0f;
    float approxRt = 0.0f;
    float peakInfectedFraction = 0.0f;
    float timeToExtinction = -1.0f;
    float simTime = 0.0f;
};

class Simulation : public simfw::SimulationBase<SimulationConfig, SimulationStats, Agent> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Agent>;
    using Grid = simfw::SpatialHashGrid<std::size_t>;

    explicit Simulation(SimulationConfig config = {});
    void update(float dt);
    void reset();

    void spawn(const Vec2& position);
    const std::vector<Agent>& getAgents() const { return m_entities; }
    const Grid& getGrid() const { return m_grid; }

private:
    Grid m_grid;
    std::vector<Agent> m_previousAgents;
    float m_simTime = 0.0f;
    std::size_t m_prevInfected = 0;
    std::size_t m_newInfections = 0;

    void normalizeConfigCounts();
    void rebuildGrid();
    void updateStats();
};

} // namespace epidemic_cpu
