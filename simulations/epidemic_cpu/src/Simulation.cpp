#include "Simulation.hpp"

#include <random/Random.hpp>
#include <simulation/EntityBrush.hpp>
#include <simulation/SimulationUtils.hpp>
#include <simulation/SpatialQuery.hpp>

#include <algorithm>
#include <cmath>

namespace epidemic_cpu {
namespace {
Vec2 randomPoint(float width, float height) {
    return Vec2{Random::range(0.0f, width), Random::range(0.0f, height)};
}

Vec2 randomVelocity(float maxSpeed) {
    return Vec2{Random::range(-maxSpeed, maxSpeed), Random::range(-maxSpeed, maxSpeed)};
}

float infectionProbability(float rate, float exposureSeconds, float dt) {
    return std::clamp(rate * exposureSeconds * dt, 0.0f, 1.0f);
}
} // namespace

Simulation::Simulation(SimulationConfig config) : Base(config), m_grid(config.gridCellSize) {
    normalizeConfigCounts();
    reset();
}

void Simulation::normalizeConfigCounts() {
    simfw::simulation::syncEntityCount(SimulationConfig::DefaultAgentCount, m_config.agentCount, m_config);
}

void Simulation::reset() {
    normalizeConfigCounts();
    m_entities.clear();
    m_entities.reserve(m_config.agentCount);
    m_previousAgents.clear();
    m_previousAgents.reserve(m_config.agentCount);
    m_simTime = 0.0f;
    m_prevInfected = 0;

    for (std::size_t i = 0; i < m_config.agentCount; ++i) {
        Agent a;
        a.position = randomPoint(m_config.width, m_config.height);
        a.velocity = randomVelocity(m_config.maxSpeed * 0.5f);
        a.radius = m_config.agentRadius;
        a.infectionState = Random::range(0.0f, 1.0f) < m_config.initialInfectedRatio ? InfectionState::Infected : InfectionState::Susceptible;
        m_entities.push_back(a);
    }
    updateStats();
}

void Simulation::rebuildGrid() {
    m_grid.setCellSize(m_config.gridCellSize);
    if (m_config.execution.useSpatialGrid) {
        m_grid.build(m_previousAgents, [](const Agent& a) { return a.position; });
        m_stats.occupiedGridCells = m_grid.getCells().size();
    } else {
        m_grid.clear();
    }
}

void Simulation::update(float dt) {
    normalizeConfigCounts();
    m_stats = {};
    m_newInfections = 0;
    m_simTime += std::max(0.0f, dt);

    m_previousAgents = m_entities;
    rebuildGrid();

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        Agent& agent = m_entities[i];
        const Agent& prev = m_previousAgents[i];

        agent.velocity = prev.velocity + randomVelocity(m_config.steeringJitter) * dt;
        if (agent.velocity.length() > m_config.maxSpeed) {
            agent.velocity = agent.velocity.normalized() * m_config.maxSpeed;
        }
        agent.position = prev.position + agent.velocity * dt;
        if (agent.position.x < 0.0f) {
            agent.position.x = 0.0f;
            agent.velocity.x = std::abs(agent.velocity.x);
        } else if (agent.position.x > m_config.width) {
            agent.position.x = m_config.width;
            agent.velocity.x = -std::abs(agent.velocity.x);
        }

        if (agent.position.y < 0.0f) {
            agent.position.y = 0.0f;
            agent.velocity.y = std::abs(agent.velocity.y);
        } else if (agent.position.y > m_config.height) {
            agent.position.y = m_config.height;
            agent.velocity.y = -std::abs(agent.velocity.y);
        }
        agent.infectedSeconds = prev.infectedSeconds;
        agent.exposureSeconds = prev.exposureSeconds;

        if (prev.infectionState == InfectionState::Infected) {
            agent.infectedSeconds += dt;
            if (agent.infectedSeconds >= m_config.recoveryTime) {
                agent.infectionState = InfectionState::Recovered;
            }
        }

        if (prev.infectionState == InfectionState::Susceptible) {
            std::vector<std::size_t> candidates;
            simfw::simulation::collectCandidates(
                m_grid,
                prev.position,
                m_config.infectionRadius,
                simfw::simulation::makeSpatialQueryOptionsExcluding(m_config.execution.useSpatialGrid, m_previousAgents.size(), i),
                candidates
            );
            m_stats.neighborCandidates += candidates.size();

            float exposure = 0.0f;
            for (std::size_t candidateIndex : candidates) {
                const Agent& other = m_previousAgents[candidateIndex];
                if (other.infectionState != InfectionState::Infected) {
                    continue;
                }
                ++m_stats.neighborChecks;
                const float distance = (other.position - prev.position).length();
                if (distance <= m_config.infectionRadius) {
                    exposure += (1.0f - (distance / std::max(0.001f, m_config.infectionRadius)));
                }
            }

            agent.exposureSeconds = exposure;
            if (exposure > 0.0f && Random::range(0.0f, 1.0f) < infectionProbability(m_config.infectionRate, exposure, dt)) {
                agent.infectionState = InfectionState::Infected;
                agent.infectedSeconds = 0.0f;
                ++m_newInfections;
            }
        }
    }

    updateStats();
}

void Simulation::updateStats() {
    m_stats.agentCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
    m_config.entityCount = m_entities.size();
    m_stats.simTime = m_simTime;

    for (const Agent& a : m_entities) {
        if (a.infectionState == InfectionState::Susceptible) ++m_stats.susceptibleCount;
        else if (a.infectionState == InfectionState::Infected) ++m_stats.infectedCount;
        else ++m_stats.recoveredCount;
    }

    if (m_stats.agentCount > 0) {
        m_stats.infectedFraction = static_cast<float>(m_stats.infectedCount) / static_cast<float>(m_stats.agentCount);
    }
    m_stats.peakInfectedFraction = std::max(m_stats.peakInfectedFraction, m_stats.infectedFraction);
    if (m_stats.infectedCount == 0 && m_stats.timeToExtinction < 0.0f) {
        m_stats.timeToExtinction = m_simTime;
    }

    m_stats.approxRt = m_prevInfected > 0 ? static_cast<float>(m_newInfections) / static_cast<float>(m_prevInfected) : 0.0f;
    m_prevInfected = m_stats.infectedCount;
}

void Simulation::spawn(const Vec2& position) {
    simfw::simulation::spawnBrush(m_entities.size(), m_config.maxAgentCount, m_config.spawnCount, [this, position]() {
        Agent a;
        a.position = position + simfw::simulation::randomDiscOffset(m_config.brushRadius);
        a.position.x = std::clamp(a.position.x, 0.0f, m_config.width);
        a.position.y = std::clamp(a.position.y, 0.0f, m_config.height);
        a.velocity = randomVelocity(m_config.maxSpeed * 0.5f);
        a.radius = m_config.agentRadius;
        m_entities.push_back(a);
    });
    updateStats();
}

} // namespace epidemic_cpu
