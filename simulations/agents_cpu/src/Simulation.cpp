#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

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

void resolveObstacleOverlap(Agent& agent, const std::vector<Obstacle>& obstacles) {
    for (const Obstacle& obstacle : obstacles) {
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
}

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

Vec2 Simulation::randomPoint() const {
    return Vec2{
        Random::range(0.0f, m_config.width),
        Random::range(0.0f, m_config.height)
    };
}

float Simulation::maxObstacleQueryRadius() const {
    float maxRadius = 0.0f;

    for (const Obstacle& obstacle : m_obstacles) {
        maxRadius = std::max(maxRadius, obstacle.radius);
    }

    return maxRadius + m_config.obstacleAvoidanceRadius;
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

void Simulation::update(float dt) {
    beginFrame();
    snapshotAgents();
    buildSpatialIndexes();
    updateAgents(dt);
}

void Simulation::updateAgents(float dt) {
    std::vector<std::size_t> agentCandidates;
    std::vector<std::size_t> obstacleCandidates;

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        updateAgent(i, dt, agentCandidates, obstacleCandidates);
    }
}

void Simulation::updateAgent(
    std::size_t agentIndex,
    float dt,
    std::vector<std::size_t>& agentCandidates,
    std::vector<std::size_t>& obstacleCandidates
) {
    Agent& agent = m_entities[agentIndex];
    const Agent& previousAgent = m_previousAgents[agentIndex];

    if (m_config.useSpatialGrid) {
        agentCandidates.clear();
        m_agentGrid.queryRadius(
            previousAgent.position,
            m_config.separationRadius,
            agentCandidates
        );
        m_stats.neighborCandidates += agentCandidates.size();

        obstacleCandidates.clear();
        m_obstacleGrid.queryRadius(
            previousAgent.position,
            maxObstacleQueryRadius(),
            obstacleCandidates
        );
        m_stats.obstacleCandidates += obstacleCandidates.size();
    } else {
        collectNaiveCandidates(agentIndex, m_previousAgents.size(), agentCandidates);
        collectAllCandidates(m_obstacles.size(), obstacleCandidates);
    }

    const Vec2 seek = seekForce(
        previousAgent,
        previousAgent.target,
        m_config.maxSpeed,
        m_config.maxForce,
        m_config.arrivalRadius
    ) * m_config.seekWeight;

    const Vec2 separation = separationForce(
        agentIndex,
        m_previousAgents,
        agentCandidates,
        m_config.separationRadius,
        m_stats
    ) * (m_config.maxForce * m_config.separationWeight);

    const Vec2 obstacleAvoidance = obstacleAvoidanceForce(
        previousAgent,
        m_obstacles,
        obstacleCandidates,
        m_config.obstacleAvoidanceRadius,
        m_stats
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

    resolveObstacleOverlap(agent, m_obstacles);

    agent.position = clampToWorld(
        agent.position,
        m_config.width,
        m_config.height
    );

    if ((agent.target - agent.position).length() <= m_config.targetRadius) {
        ++m_stats.arrivedCount;
    }
}

} // namespace agents_cpu
