#pragma once

#include "Agent.hpp"
#include "CrowdBehavior.hpp"
#include "Obstacle.hpp"

#include <simulation/NeighborScratch.hpp>
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
    std::size_t entityCount = DefaultAgentCount;
    float agentRadius = 4.0f;
    float maxSpeed = 120.0f;
    float maxForce = 160.0f;
    float flowWeight = 1.0f;
    float separationWeight = 2.0f;
    float obstacleAvoidanceWeight = 2.5f;
    float separationRadius = 16.0f;
    float obstacleRadius = 24.0f;
    float obstacleAvoidanceRadius = 58.0f;
    float goalRadius = 14.0f;
    float gridCellSize = 24.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
};

struct SimulationStats {
    std::size_t agentCount = 0;
    std::size_t entityCount = 0;
    std::size_t obstacleCount = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t occupiedObstacleGridCells = 0;
    std::size_t neighborCandidates = 0;
    std::size_t neighborChecks = 0;
    std::size_t obstacleCandidates = 0;
    std::size_t obstacleChecks = 0;
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
    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    void update(float dt);
    void reset();
    void setGoal(Vec2 goal);
    void addObstacle(Vec2 pos);
    void clearObstacles();

    Vec2 getGoal() const { return m_goal; }
    const Grid& getGrid() const { return m_agentGrid; }
    const Grid& getObstacleGrid() const { return m_obstacleGrid; }
    const std::vector<Obstacle>& getObstacles() const { return m_obstacles; }
    const std::vector<float>& getIntegrationField() const { return m_integrationField; }
    const std::vector<Vec2>& getFlowField() const { return m_flowField; }

private:
    using Scratch = simfw::simulation::NeighborScratch<std::size_t>;
    Grid m_agentGrid;
    Grid m_obstacleGrid;
    std::vector<Agent> m_previousAgents;
    std::vector<Obstacle> m_obstacles;
    Vec2 m_goal;
    std::unique_ptr<ThreadPool> m_threadPool;
    std::vector<WeightedBehavior> m_behaviors;

    std::size_t m_gridWidth = 0;
    std::size_t m_gridHeight = 0;
    std::vector<float> m_costField;
    std::vector<float> m_integrationField;
    std::vector<Vec2> m_flowField;

    void beginFrame();
    void buildSpatialIndexes();
    void buildFlowField();
    void updateAgents(float dt);
};

} // namespace crowd_cpu
