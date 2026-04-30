#pragma once

#include "Agent.hpp"
#include "CrowdBehavior.hpp"

#include <simulation/NeighborScratch.hpp>
#include <simulation/ObstacleMask.hpp>
#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <memory>
#include <vector>

class ThreadPool;

namespace crowd_cpu {

struct SimulationConfig {
    static constexpr std::size_t DefaultAgentCount = 300;
    float width = 800.0f;
    float height = 800.0f;
    std::size_t agentCount = DefaultAgentCount;
    std::size_t maxAgentCount = 3000;
    int spawnCount = 6;
    float brushRadius = 20.0f;
    std::size_t entityCount = DefaultAgentCount;
    float agentRadius = 4.0f;
    float maxSpeed = 120.0f;
    float maxForce = 160.0f;
    float flowWeight = 1.0f;
    float separationWeight = 2.0f;
    float obstacleAvoidanceWeight = 0.0f;
    float separationRadius = 16.0f;
    float goalRadius = 14.0f;
    float gridCellSize = 24.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
};

struct SimulationStats {
    std::size_t agentCount = 0;
    std::size_t entityCount = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t neighborCandidates = 0;
    std::size_t neighborChecks = 0;
    std::size_t reachedGoalCount = 0;
};

class Simulation : public simfw::SimulationBase<SimulationConfig, SimulationStats, Agent> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Agent>;
    using Grid = simfw::SpatialHashGrid<std::size_t>;
    explicit Simulation(SimulationConfig config = {});
    ~Simulation();
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    void update(float dt);
    void reset();
    void setGoal(Vec2 goal);
    void spawn(const Vec2& position);

    simfw::simulation::ObstacleMask& obstacleMask() { return m_obstacleMask; }
    const simfw::simulation::ObstacleMask& obstacleMask() const { return m_obstacleMask; }

    Vec2 getGoal() const { return m_goal; }
    const Grid& getGrid() const { return m_agentGrid; }
    const std::vector<float>& getCostField() const { return m_costField; }
    const std::vector<float>& getIntegrationField() const { return m_integrationField; }
    Vec2 sampleFlow(Vec2 worldPos) const;

private:
    using Scratch = simfw::simulation::NeighborScratch<std::size_t>;
    Grid m_agentGrid;
    std::vector<Agent> m_previousAgents;
    Vec2 m_goal;
    std::unique_ptr<ThreadPool> m_threadPool;
    std::vector<WeightedBehavior> m_behaviors;
    simfw::simulation::ObstacleMask m_obstacleMask;

    std::size_t m_gridWidth = 0;
    std::size_t m_gridHeight = 0;
    std::vector<float> m_costField;
    std::vector<float> m_integrationField;
    void beginFrame();
    void buildSpatialIndexes();
    void buildFlowField();
    void updateAgents(float dt);
    bool isBlockedWorld(Vec2 worldPos) const;
};

} // namespace crowd_cpu
