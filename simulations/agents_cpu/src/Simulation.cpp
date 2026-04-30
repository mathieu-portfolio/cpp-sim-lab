#include "Simulation.hpp"

#include "thread_pool.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random/Random.hpp>
#include <simulation/ParallelUpdate.hpp>
#include <simulation/SpatialQuery.hpp>
#include <simulation/StatsReduction.hpp>
#include <simulation/EntityBrush.hpp>
#include <thread>

namespace agents_cpu {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;

Vec2 agentPosition(const Agent& agent) {
    return agent.position;
}

Vec2 obstaclePosition(const Obstacle& obstacle) {
    return obstacle.position;
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
      m_threadPool(std::make_unique<ThreadPool>(hardwareWorkerCount())),
      m_behaviors(steering::makeDefaultBehaviors()),
      m_intentRules(steering::makeDefaultIntentRules()) {
    normalizeConfigCounts();
    reset();
}

Simulation::~Simulation() = default;

Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::setBehaviors(BehaviorList behaviors) {
    m_behaviors = std::move(behaviors);
}

void Simulation::resetBehaviors() {
    m_behaviors = steering::makeDefaultBehaviors();
}

void Simulation::setIntentRules(IntentRuleList intentRules) {
    m_intentRules = std::move(intentRules);
}

void Simulation::resetIntentRules() {
    m_intentRules = steering::makeDefaultIntentRules();
}

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
    m_config.entityCount = m_entities.size();
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


void Simulation::spawn(const Vec2& position) {
    const Vec2 clamped = clampToWorld(position, m_config.width, m_config.height);
    simfw::simulation::spawnBrush(m_entities.size(), m_config.maxAgentCount, m_config.spawnCount, [this, clamped]() {
        const Vec2 spawnPos = clampToWorld(clamped + simfw::simulation::randomDiscOffset(m_config.brushRadius), m_config.width, m_config.height);
        m_entities.push_back({spawnPos, Vec2{Random::range(-20.0f, 20.0f), Random::range(-20.0f, 20.0f)}, m_target, m_config.agentRadius, AgentIntent::SeekTarget});
    });
    updateStatsCount();
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

    if (m_config.execution.useSpatialGrid) {
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
    simfw::simulation::collectCandidates(
        m_agentGrid,
        m_previousAgents[agentIndex].position,
        m_config.separationRadius,
        simfw::simulation::makeSpatialQueryOptionsExcluding(
            m_config.execution.useSpatialGrid,
            m_previousAgents.size(),
            agentIndex
        ),
        scratch.candidates
    );

    if (m_config.execution.useSpatialGrid) {
        stats.neighborCandidates += scratch.candidates.size();
    }
}

void Simulation::collectObstacleCandidates(
    const Agent& previousAgent,
    float obstacleQueryRadius,
    AgentUpdateScratch& scratch,
    SimulationStats& stats
) {
    simfw::simulation::collectCandidates(
        m_obstacleGrid,
        previousAgent.position,
        obstacleQueryRadius,
        simfw::simulation::makeSpatialQueryOptions(
            m_config.execution.useSpatialGrid,
            m_obstacles.size()
        ),
        scratch.secondaryNeighbors
    );

    if (m_config.execution.useSpatialGrid) {
        stats.obstacleCandidates += scratch.secondaryNeighbors.size();
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

        steering::BehaviorContext behaviorContext{
            m_config,
            m_previousAgents,
            m_obstacles,
            steering::CandidateLists{
                scratch.candidates,
                scratch.secondaryNeighbors
            },
            previousAgent.intent,
            stats
        };

        const AgentIntent intent = steering::selectIntent(
            i,
            behaviorContext,
            m_intentRules
        );

        steering::recordIntentStats(intent, previousAgent.intent, stats);
        behaviorContext.intent = intent;

        const Vec2 acceleration = steering::computeAcceleration(
            i,
            behaviorContext,
            m_behaviors
        );

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
            scratch.secondaryNeighbors,
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
    simfw::sumStatsMembers(
        m_stats,
        workerStats,
        &SimulationStats::neighborChecks,
        &SimulationStats::neighborCandidates,
        &SimulationStats::arrivedCount,
        &SimulationStats::obstacleChecks,
        &SimulationStats::obstacleCandidates,
        &SimulationStats::obstacleOverlapChecks,
        &SimulationStats::seekingTargetCount,
        &SimulationStats::avoidingObstacleCount,
        &SimulationStats::idleCount,
        &SimulationStats::intentChanges
    );
}

void Simulation::updateAgents(float dt) {
    simfw::runParallelUpdate<ThreadPool, AgentUpdateScratch, SimulationStats>(
        m_threadPool.get(),
        m_entities.size(),
        MinItemsPerParallelTask,
        m_config.execution.useParallelUpdate,
        [this, dt, obstacleQueryRadius = maxObstacleQueryRadius(dt)](
            std::size_t beginIndex,
            std::size_t endIndex,
            AgentUpdateScratch& scratch,
            SimulationStats& stats
        ) {
            updateAgentRange(
                beginIndex,
                endIndex,
                dt,
                obstacleQueryRadius,
                scratch,
                stats
            );
        },
        [this](const SimulationStats& stats) {
            mergeWorkerStats(stats);
        }
    );
}

void Simulation::update(float dt) {
    beginFrame();
    snapshotAgents();
    buildSpatialIndexes();
    updateAgents(dt);
}

} // namespace agents_cpu
