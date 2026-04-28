#include "Simulation.hpp"

#include "SteeringBehaviors.hpp"
#include "thread_pool.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random/Random.hpp>
#include <thread>

namespace agents_cpu {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;
constexpr float Epsilon = 0.0001f;

Vec2 agentPosition(const Agent& agent) {
    return agent.position;
}

Vec2 obstaclePosition(const Obstacle& obstacle) {
    return obstacle.position;
}

void collectNaiveCandidates(
    std::size_t excludedIndex,
    std::size_t count,
    std::vector<std::size_t>& out
) {
    out.clear();
    out.reserve(count > 0 ? count - 1 : 0);

    for (std::size_t i = 0; i < count; ++i) {
        if (i != excludedIndex) {
            out.push_back(i);
        }
    }
}

void collectAllCandidates(std::size_t count, std::vector<std::size_t>& out) {
    out.clear();
    out.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        out.push_back(i);
    }
}

Vec2 clampToWorld(Vec2 position, float width, float height) {
    position.x = std::clamp(position.x, 0.0f, width);
    position.y = std::clamp(position.y, 0.0f, height);
    return position;
}

std::size_t hardwareWorkerCount() {
    const unsigned int hardwareThreads = std::thread::hardware_concurrency();

    if (hardwareThreads == 0) {
        return 1;
    }

    return static_cast<std::size_t>(hardwareThreads);
}

bool hasObstacleThreat(
    const Agent& agent,
    const std::vector<Obstacle>& obstacles,
    const std::vector<std::size_t>& obstacleCandidates,
    float obstacleIntentRadius
) {
    for (std::size_t obstacleIndex : obstacleCandidates) {
        const Obstacle& obstacle = obstacles[obstacleIndex];
        const Vec2 away = agent.position - obstacle.position;
        const float distance = away.length();
        const float threatDistance = obstacle.radius + agent.radius + obstacleIntentRadius;

        if (distance > Epsilon && distance <= threatDistance) {
            return true;
        }
    }

    return false;
}

AgentIntent decideIntent(
    const Agent& previousAgent,
    const SimulationConfig& config,
    const std::vector<Obstacle>& obstacles,
    const std::vector<std::size_t>& obstacleCandidates
) {
    if (!config.useIntent) {
        return AgentIntent::SeekTarget;
    }

    if (hasObstacleThreat(
            previousAgent,
            obstacles,
            obstacleCandidates,
            config.obstacleIntentRadius
        )) {
        return AgentIntent::AvoidObstacle;
    }

    if ((previousAgent.target - previousAgent.position).length() <= config.targetRadius) {
        return AgentIntent::Idle;
    }

    return AgentIntent::SeekTarget;
}

void recordIntentStats(
    AgentIntent intent,
    AgentIntent previousIntent,
    SimulationStats& stats
) {
    switch (intent) {
    case AgentIntent::SeekTarget:
        ++stats.seekingTargetCount;
        break;
    case AgentIntent::AvoidObstacle:
        ++stats.avoidingObstacleCount;
        break;
    case AgentIntent::Idle:
        ++stats.idleCount;
        break;
    }

    if (intent != previousIntent) {
        ++stats.intentChanges;
    }
}

float dampingFactor(float damping, float dt) {
    if (damping <= 0.0f) {
        return 1.0f;
    }

    return std::max(0.0f, 1.0f - damping * std::max(dt, 0.0f));
}
} // namespace

Simulation::Simulation(SimulationConfig config)
    : Base(config),
      m_agentGrid(config.gridCellSize),
      m_obstacleGrid(config.gridCellSize),
      m_target{config.width * 0.5f, config.height * 0.5f},
      m_threadPool(std::make_unique<ThreadPool>(hardwareWorkerCount())) {
    normalizeConfigCounts();
    reset();
}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::normalizeConfigCounts() {
    constexpr std::size_t defaultCount = SimulationConfig::DefaultAgentCount;

    if (m_config.agentCount == defaultCount && m_config.entityCount != defaultCount) {
        m_config.agentCount = m_config.entityCount;
    } else {
        m_config.entityCount = m_config.agentCount;
    }
}

void Simulation::updateStatsCount() {
    m_stats.agentCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
    m_stats.obstacleCount = m_obstacles.size();
}

Vec2 Simulation::randomPoint() const {
    return Vec2{
        Random::range(0.0f, m_config.width),
        Random::range(0.0f, m_config.height)
    };
}

float Simulation::maxObstacleQueryRadius(float dt) const {
    float maxRadius = 0.0f;

    for (const Obstacle& obstacle : m_obstacles) {
        maxRadius = std::max(maxRadius, obstacle.radius);
    }

    const float maxMoveDistance = m_config.maxSpeed * std::max(dt, 0.0f);
    const float overlapRadius = m_config.agentRadius + maxMoveDistance;
    const float agentInfluenceRadius = std::max(
        std::max(m_config.obstacleAvoidanceRadius, m_config.obstacleIntentRadius),
        overlapRadius
    );

    return maxRadius + agentInfluenceRadius;
}

void Simulation::reset() {
    normalizeConfigCounts();

    m_entities.clear();
    m_entities.reserve(m_config.agentCount);
    m_previousAgents.clear();
    m_previousAgents.reserve(m_config.agentCount);

    m_target = Vec2{m_config.width * 0.5f, m_config.height * 0.5f};

    for (std::size_t i = 0; i < m_config.agentCount; ++i) {
        const Vec2 position = randomPoint();

        m_entities.push_back({
            position,
            Vec2{
                Random::range(-20.0f, 20.0f),
                Random::range(-20.0f, 20.0f)
            },
            m_target,
            m_config.agentRadius,
            AgentIntent::SeekTarget
        });
    }

    m_stats = {};
    updateStatsCount();
}

void Simulation::setTarget(Vec2 target) {
    m_target = clampToWorld(target, m_config.width, m_config.height);

    for (Agent& agent : m_entities) {
        agent.target = m_target;

        if (agent.intent == AgentIntent::Idle) {
            agent.intent = AgentIntent::SeekTarget;
        }
    }
}

void Simulation::addObstacle(Vec2 position) {
    m_obstacles.push_back({
        clampToWorld(position, m_config.width, m_config.height),
        m_config.obstacleRadius
    });

    updateStatsCount();
}

void Simulation::clearObstacles() {
    m_obstacles.clear();
    updateStatsCount();
}

void Simulation::beginFrame() {
    normalizeConfigCounts();

    m_stats = {};
    updateStatsCount();
}

void Simulation::snapshotAgents() {
    m_previousAgents = m_entities;
}

void Simulation::buildSpatialIndexes() {
    m_agentGrid.setCellSize(m_config.gridCellSize);
    m_obstacleGrid.setCellSize(m_config.gridCellSize);

    if (m_config.useSpatialGrid) {
        m_agentGrid.build(m_previousAgents, agentPosition);
        m_obstacleGrid.build(m_obstacles, obstaclePosition);
        m_stats.occupiedGridCells = m_agentGrid.getCells().size();
        m_stats.occupiedObstacleGridCells = m_obstacleGrid.getCells().size();
    } else {
        m_agentGrid.clear();
        m_obstacleGrid.clear();
    }
}

void Simulation::collectAgentCandidates(
    std::size_t agentIndex,
    AgentUpdateScratch& scratch,
    SimulationStats& stats
) {
    auto& candidates = scratch.agentCandidates;

    if (m_config.useSpatialGrid) {
        candidates.clear();
        m_agentGrid.queryRadius(
            m_previousAgents[agentIndex].position,
            m_config.separationRadius,
            candidates
        );
        stats.neighborCandidates += candidates.size();
    } else {
        collectNaiveCandidates(agentIndex, m_previousAgents.size(), candidates);
    }
}

void Simulation::collectObstacleCandidates(
    const Agent& previousAgent,
    float obstacleQueryRadius,
    AgentUpdateScratch& scratch,
    SimulationStats& stats
) {
    auto& candidates = scratch.obstacleCandidates;

    if (m_config.useSpatialGrid) {
        candidates.clear();
        m_obstacleGrid.queryRadius(
            previousAgent.position,
            obstacleQueryRadius,
            candidates
        );
        stats.obstacleCandidates += candidates.size();
    } else {
        collectAllCandidates(m_obstacles.size(), candidates);
    }
}

void Simulation::updateAgentRange(
    std::size_t beginIndex,
    std::size_t endIndex,
    float dt,
    float obstacleQueryRadius,
    AgentUpdateScratch& scratch,
    SimulationStats& stats
) {
    for (std::size_t i = beginIndex; i < endIndex; ++i) {
        Agent& agent = m_entities[i];
        const Agent& previousAgent = m_previousAgents[i];

        collectAgentCandidates(i, scratch, stats);
        collectObstacleCandidates(previousAgent, obstacleQueryRadius, scratch, stats);

        const AgentIntent intent = decideIntent(
            previousAgent,
            m_config,
            m_obstacles,
            scratch.obstacleCandidates
        );

        recordIntentStats(intent, previousAgent.intent, stats);

        steering::BehaviorContext behaviorContext{
            m_config,
            m_previousAgents,
            m_obstacles,
            steering::CandidateLists{
                scratch.agentCandidates,
                scratch.obstacleCandidates
            },
            intent,
            stats
        };

        const Vec2 acceleration = steering::computeAcceleration(i, behaviorContext);

        agent.velocity = previousAgent.velocity + acceleration * dt;

        if (intent == AgentIntent::Idle) {
            agent.velocity *= dampingFactor(m_config.idleDamping, dt);
        }

        agent.velocity = steering::limitLength(agent.velocity, m_config.maxSpeed);
        agent.position = previousAgent.position + agent.velocity * dt;
        agent.target = previousAgent.target;
        agent.radius = previousAgent.radius;
        agent.intent = intent;

        steering::resolveObstacleOverlap(
            agent,
            m_obstacles,
            scratch.obstacleCandidates,
            stats
        );

        agent.position = clampToWorld(
            agent.position,
            m_config.width,
            m_config.height
        );

        if ((agent.target - agent.position).length() <= m_config.targetRadius) {
            ++stats.arrivedCount;
        }
    }
}

void Simulation::mergeWorkerStats(const SimulationStats& workerStats) {
    m_stats.neighborChecks += workerStats.neighborChecks;
    m_stats.neighborCandidates += workerStats.neighborCandidates;
    m_stats.arrivedCount += workerStats.arrivedCount;

    m_stats.obstacleChecks += workerStats.obstacleChecks;
    m_stats.obstacleCandidates += workerStats.obstacleCandidates;
    m_stats.obstacleOverlapChecks += workerStats.obstacleOverlapChecks;

    m_stats.seekingTargetCount += workerStats.seekingTargetCount;
    m_stats.avoidingObstacleCount += workerStats.avoidingObstacleCount;
    m_stats.idleCount += workerStats.idleCount;
    m_stats.intentChanges += workerStats.intentChanges;
}

void Simulation::updateAgentsSingleThread(float dt, float obstacleQueryRadius) {
    AgentUpdateScratch scratch;
    SimulationStats stats;

    updateAgentRange(
        0,
        m_entities.size(),
        dt,
        obstacleQueryRadius,
        scratch,
        stats
    );

    mergeWorkerStats(stats);
}

void Simulation::updateAgentsParallel(float dt, float obstacleQueryRadius) {
    const std::size_t agentCount = m_entities.size();
    const std::size_t taskCount =
        (agentCount + MinItemsPerParallelTask - 1) / MinItemsPerParallelTask;

    std::vector<AgentUpdateScratch> workerScratch(taskCount);
    std::vector<SimulationStats> workerStats(taskCount);

    m_threadPool->parallel_for(
        agentCount,
        MinItemsPerParallelTask,
        [this, dt, obstacleQueryRadius, &workerScratch, &workerStats](
            std::size_t beginIndex,
            std::size_t endIndex,
            std::size_t taskIndex
        ) {
            updateAgentRange(
                beginIndex,
                endIndex,
                dt,
                obstacleQueryRadius,
                workerScratch[taskIndex],
                workerStats[taskIndex]
            );
        }
    );

    for (const SimulationStats& stats : workerStats) {
        mergeWorkerStats(stats);
    }
}

void Simulation::updateAgents(float dt) {
    if (m_entities.empty()) {
        return;
    }

    const float obstacleQueryRadius = maxObstacleQueryRadius(dt);

    if (m_config.useParallelUpdate) {
        updateAgentsParallel(dt, obstacleQueryRadius);
    } else {
        updateAgentsSingleThread(dt, obstacleQueryRadius);
    }
}

void Simulation::update(float dt) {
    beginFrame();
    snapshotAgents();
    buildSpatialIndexes();
    updateAgents(dt);
}

} // namespace agents_cpu
