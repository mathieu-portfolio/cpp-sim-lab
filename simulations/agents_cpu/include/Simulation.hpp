#pragma once

#include "Agent.hpp"
#include "Obstacle.hpp"

#include <simulation/SimulationBase.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <memory>
#include <vector>

class ThreadPool;

namespace agents_cpu {

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
    float obstacleAvoidanceWeight = 3.0f;

    float arrivalRadius = 80.0f;
    float targetRadius = 12.0f;
    float separationRadius = 18.0f;

    float obstacleRadius = 24.0f;
    float obstacleAvoidanceRadius = 70.0f;

    bool useSpatialGrid = true;
    bool useParallelUpdate = true;
    float gridCellSize = 24.0f;
};

struct SimulationStats {
    std::size_t agentCount = 0;
    std::size_t entityCount = 0;

    std::size_t neighborChecks = 0;
    std::size_t neighborCandidates = 0;
    std::size_t occupiedGridCells = 0;
    std::size_t arrivedCount = 0;

    std::size_t obstacleCount = 0;
    std::size_t obstacleChecks = 0;
    std::size_t obstacleCandidates = 0;
    std::size_t obstacleOverlapChecks = 0;
    std::size_t occupiedObstacleGridCells = 0;
};

class Simulation
    : public simfw::SimulationBase<SimulationConfig, SimulationStats, Agent> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Agent>;
    using Grid = simfw::SpatialHashGrid<std::size_t>;

    explicit Simulation(SimulationConfig config = {});
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) noexcept;
    Simulation& operator=(Simulation&&) noexcept;

    void update(float dt);
    void reset();

    const std::vector<Agent>& getAgents() const { return m_entities; }
    const std::vector<Obstacle>& getObstacles() const { return m_obstacles; }

    const Grid& getGrid() const { return m_agentGrid; }
    const Grid& getAgentGrid() const { return m_agentGrid; }
    const Grid& getObstacleGrid() const { return m_obstacleGrid; }

    void setTarget(Vec2 target);
    Vec2 getTarget() const { return m_target; }

    void addObstacle(Vec2 position);
    void clearObstacles();

private:
    struct AgentUpdateScratch {
        std::vector<std::size_t> agentCandidates;
        std::vector<std::size_t> obstacleCandidates;
    };

    Grid m_agentGrid;
    Grid m_obstacleGrid;
    Vec2 m_target;
    std::vector<Obstacle> m_obstacles;
    std::vector<Agent> m_previousAgents;
    std::unique_ptr<ThreadPool> m_threadPool;

    void normalizeConfigCounts();
    void updateStatsCount();
    Vec2 randomPoint() const;
    float maxObstacleQueryRadius(float dt) const;

    void beginFrame();
    void snapshotAgents();
    void buildSpatialIndexes();
    void updateAgents(float dt);
    void updateAgentsSingleThread(float dt, float obstacleQueryRadius);
    void updateAgentsParallel(float dt, float obstacleQueryRadius);
    void updateAgentRange(
        std::size_t beginIndex,
        std::size_t endIndex,
        float dt,
        float obstacleQueryRadius,
        AgentUpdateScratch& scratch,
        SimulationStats& stats
    );
    void collectAgentCandidates(
        std::size_t agentIndex,
        AgentUpdateScratch& scratch,
        SimulationStats& stats
    );
    void collectObstacleCandidates(
        const Agent& previousAgent,
        float obstacleQueryRadius,
        AgentUpdateScratch& scratch,
        SimulationStats& stats
    );
    void mergeWorkerStats(const SimulationStats& workerStats);
};

} // namespace agents_cpu
