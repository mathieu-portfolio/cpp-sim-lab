#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

#include <thread>

namespace agents_cpu {
namespace {
constexpr float Epsilon = 0.0001f;

Vec2 agentPosition(const Agent& agent) {
    return agent.position;
}

Vec2 obstaclePosition(const Obstacle& obstacle) {
    return obstacle.position;
}

Vec2 limitLength(Vec2 value, float maxLength) {
    const float length = value.length();

    if (length <= maxLength || length <= Epsilon) {
        return value;
    }

    return value * (maxLength / length);
}

Vec2 seekForce(
    const Agent& agent,
    Vec2 target,
    float maxSpeed,
    float maxForce,
    float arrivalRadius
) {
    const Vec2 toTarget = target - agent.position;
    const float distance = toTarget.length();

    if (distance <= Epsilon) {
        return Vec2{};
    }

    float desiredSpeed = maxSpeed;

    if (distance < arrivalRadius) {
        desiredSpeed *= std::clamp(distance / arrivalRadius, 0.0f, 1.0f);
    }

    const Vec2 desiredVelocity = toTarget.normalized() * desiredSpeed;
    return limitLength(desiredVelocity - agent.velocity, maxForce);
}

Vec2 separationForce(
    std::size_t agentIndex,
    const std::vector<Agent>& agents,
    const std::vector<std::size_t>& candidates,
    float separationRadius,
    SimulationStats& stats
) {
    Vec2 force{};
    int neighborCount = 0;

    for (std::size_t candidateIndex : candidates) {
        if (candidateIndex == agentIndex) {
            continue;
        }

        ++stats.neighborChecks;

        const Vec2 away = agents[agentIndex].position - agents[candidateIndex].position;
        const float distance = away.length();

        if (distance <= Epsilon || distance >= separationRadius) {
            continue;
        }

        const float strength = 1.0f - (distance / separationRadius);
        force += away.normalized() * strength;
        ++neighborCount;
    }

    if (neighborCount > 0) {
        force *= 1.0f / static_cast<float>(neighborCount);
    }

    return force;
}

Vec2 obstacleAvoidanceForce(
    const Agent& agent,
    const std::vector<Obstacle>& obstacles,
    const std::vector<std::size_t>& candidates,
    float avoidanceRadius,
    SimulationStats& stats
) {
    Vec2 force{};
    int obstacleCount = 0;

    for (std::size_t obstacleIndex : candidates) {
        ++stats.obstacleChecks;

        const Obstacle& obstacle = obstacles[obstacleIndex];
        const Vec2 away = agent.position - obstacle.position;
        const float distance = away.length();
        const float influenceDistance = obstacle.radius + avoidanceRadius;

        if (distance <= Epsilon || distance >= influenceDistance) {
            continue;
        }

        const float strength = 1.0f - (distance / influenceDistance);
        force += away.normalized() * strength;
        ++obstacleCount;
    }

    if (obstacleCount > 0) {
        force *= 1.0f / static_cast<float>(obstacleCount);
    }

    return force;
}

void resolveObstacleOverlap(
    Agent& agent,
    const std::vector<Obstacle>& obstacles,
    const std::vector<std::size_t>& candidates,
    SimulationStats& stats
) {
    for (std::size_t obstacleIndex : candidates) {
        ++stats.obstacleOverlapChecks;

        const Obstacle& obstacle = obstacles[obstacleIndex];
        Vec2 away = agent.position - obstacle.position;
        float distance = away.length();

        const float minDistance = obstacle.radius + agent.radius;

        if (distance <= Epsilon || distance >= minDistance) {
            continue;
        }

        Vec2 normal = away * (1.0f / distance);
        agent.position = obstacle.position + normal * minDistance;

        const float velocityIntoObstacle = Vec2::dot(agent.velocity, normal);

        if (velocityIntoObstacle < 0.0f) {
            agent.velocity -= normal * velocityIntoObstacle;
        }
    }
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

std::size_t workerCountFor(std::size_t itemCount) {
    constexpr std::size_t MinItemsPerWorker = 256;

    if (itemCount <= MinItemsPerWorker) {
        return 1;
    }

    const unsigned int hardwareThreads = std::thread::hardware_concurrency();
    const std::size_t availableWorkers = hardwareThreads == 0
        ? 1
        : static_cast<std::size_t>(hardwareThreads);

    const std::size_t usefulWorkers =
        (itemCount + MinItemsPerWorker - 1) / MinItemsPerWorker;

    return std::clamp<std::size_t>(usefulWorkers, 1, availableWorkers);
}
} // namespace

Simulation::Simulation(SimulationConfig config)
    : Base(config),
      m_agentGrid(config.gridCellSize),
      m_obstacleGrid(config.gridCellSize),
      m_target{config.width * 0.5f, config.height * 0.5f} {
    normalizeConfigCounts();
    reset();
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
    const float agentInfluenceRadius = std::max(m_config.obstacleAvoidanceRadius, overlapRadius);

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
            m_config.agentRadius
        });
    }

    m_stats = {};
    updateStatsCount();
}

void Simulation::setTarget(Vec2 target) {
    m_target = clampToWorld(target, m_config.width, m_config.height);

    for (Agent& agent : m_entities) {
        agent.target = m_target;
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

        const Vec2 seek = seekForce(
            previousAgent,
            previousAgent.target,
            m_config.maxSpeed,
            m_config.maxForce,
            m_config.arrivalRadius
        ) * m_config.seekWeight;

        const Vec2 separation = separationForce(
            i,
            m_previousAgents,
            scratch.agentCandidates,
            m_config.separationRadius,
            stats
        ) * (m_config.maxForce * m_config.separationWeight);

        const Vec2 obstacleAvoidance = obstacleAvoidanceForce(
            previousAgent,
            m_obstacles,
            scratch.obstacleCandidates,
            m_config.obstacleAvoidanceRadius,
            stats
        ) * (m_config.maxForce * m_config.obstacleAvoidanceWeight);

        const Vec2 acceleration = limitLength(
            seek + separation + obstacleAvoidance,
            m_config.maxForce
        );

        agent.velocity = previousAgent.velocity + acceleration * dt;
        agent.velocity = limitLength(agent.velocity, m_config.maxSpeed);
        agent.position = previousAgent.position + agent.velocity * dt;
        agent.target = previousAgent.target;
        agent.radius = previousAgent.radius;

        resolveObstacleOverlap(agent, m_obstacles, scratch.obstacleCandidates, stats);

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
    const std::size_t workerCount = workerCountFor(agentCount);

    std::vector<AgentUpdateScratch> workerScratch(workerCount);
    std::vector<SimulationStats> workerStats(workerCount);

    if (workerCount == 1) {
        updateAgentRange(
            0,
            agentCount,
            dt,
            obstacleQueryRadius,
            workerScratch[0],
            workerStats[0]
        );
        mergeWorkerStats(workerStats[0]);
        return;
    }

    std::vector<std::thread> workers;
    workers.reserve(workerCount);

    const std::size_t chunkSize = (agentCount + workerCount - 1) / workerCount;

    for (std::size_t workerIndex = 0; workerIndex < workerCount; ++workerIndex) {
        const std::size_t beginIndex = workerIndex * chunkSize;
        const std::size_t endIndex = std::min(beginIndex + chunkSize, agentCount);

        if (beginIndex >= endIndex) {
            break;
        }

        workers.emplace_back(
            [this, beginIndex, endIndex, dt, obstacleQueryRadius, workerIndex, &workerScratch, &workerStats]() {
                updateAgentRange(
                    beginIndex,
                    endIndex,
                    dt,
                    obstacleQueryRadius,
                    workerScratch[workerIndex],
                    workerStats[workerIndex]
                );
            }
        );
    }

    for (std::thread& worker : workers) {
        worker.join();
    }

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
