#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

namespace {
constexpr float Epsilon = 0.0001f;

Vec2 agentPosition(const Agent& agent) {
    return agent.position;
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
    float avoidanceRadius,
    SimulationStats& stats
) {
    Vec2 force{};
    int obstacleCount = 0;

    for (const Obstacle& obstacle : obstacles) {
        ++stats.obstacleChecks;

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

void collectNaiveCandidates(
    std::size_t agentIndex,
    std::size_t agentCount,
    std::vector<std::size_t>& out
) {
    out.clear();
    out.reserve(agentCount > 0 ? agentCount - 1 : 0);

    for (std::size_t i = 0; i < agentCount; ++i) {
        if (i != agentIndex) {
            out.push_back(i);
        }
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
      m_grid(config.gridCellSize),
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

void Simulation::reset() {
    normalizeConfigCounts();

    m_entities.clear();
    m_entities.reserve(m_config.agentCount);

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
    normalizeConfigCounts();

    m_stats = {};
    updateStatsCount();

    m_grid.setCellSize(m_config.gridCellSize);

    if (m_config.useSpatialGrid) {
        m_grid.build(m_entities, agentPosition);
        m_stats.occupiedGridCells = m_grid.getCells().size();
    } else {
        m_grid.clear();
    }

    std::vector<std::size_t> candidates;

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        Agent& agent = m_entities[i];

        if (m_config.useSpatialGrid) {
            candidates.clear();
            m_grid.queryRadius(agent.position, m_config.separationRadius, candidates);
            m_stats.neighborCandidates += candidates.size();
        } else {
            collectNaiveCandidates(i, m_entities.size(), candidates);
        }

        const Vec2 seek = seekForce(
            agent,
            agent.target,
            m_config.maxSpeed,
            m_config.maxForce,
            m_config.arrivalRadius
        ) * m_config.seekWeight;

        const Vec2 separation = separationForce(
            i,
            m_entities,
            candidates,
            m_config.separationRadius,
            m_stats
        ) * (m_config.maxForce * m_config.separationWeight);

        const Vec2 obstacleAvoidance = obstacleAvoidanceForce(
            agent,
            m_obstacles,
            m_config.obstacleAvoidanceRadius,
            m_stats
        ) * (m_config.maxForce * m_config.obstacleAvoidanceWeight);

        const Vec2 acceleration = limitLength(
            seek + separation + obstacleAvoidance,
            m_config.maxForce
        );

        agent.velocity += acceleration * dt;
        agent.velocity = limitLength(agent.velocity, m_config.maxSpeed);
        agent.position += agent.velocity * dt;

        agent.position = clampToWorld(
            agent.position,
            m_config.width,
            m_config.height
        );

        if ((agent.target - agent.position).length() <= m_config.targetRadius) {
            ++m_stats.arrivedCount;
        }
    }
}
