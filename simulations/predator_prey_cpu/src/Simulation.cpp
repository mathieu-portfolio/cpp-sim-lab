#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include <random/Random.hpp>
#include <simulation/EntityBrush.hpp>
#include <simulation/SimulationUtils.hpp>

#include <algorithm>
#include <cmath>

namespace predator_prey_cpu {
namespace {
Vec2 limitLength(Vec2 value, float maxLength) {
    const float lengthSquared = value.lengthSquared();
    if (lengthSquared <= 0.0001f || lengthSquared <= maxLength * maxLength) {
        return value;
    }

    return value * (maxLength / std::sqrt(lengthSquared));
}

Vec2 steerTowards(const Vec2& desired, const Vec2& currentVelocity, float maxSpeed) {
    if (desired.lengthSquared() <= 0.0001f) {
        return {};
    }

    return desired.normalized() * maxSpeed - currentVelocity;
}

Vec2 wrap(const Vec2& p, const SimulationConfig& config) {
    Vec2 wrapped = p;
    if (wrapped.x < 0.0f) wrapped.x += config.width;
    if (wrapped.x >= config.width) wrapped.x -= config.width;
    if (wrapped.y < 0.0f) wrapped.y += config.height;
    if (wrapped.y >= config.height) wrapped.y -= config.height;
    return wrapped;
}
}

Simulation::Simulation(SimulationConfig config) : simfw::SimulationBase<SimulationConfig, SimulationStats, Boid>(config), m_grid(config.gridCellSize) {
    reset();
}

void Simulation::updateStats() {
    m_stats = {};
    for (const Boid& boid : m_entities) {
        if (!boid.alive) continue;
        if (boid.type == AgentType::Prey) ++m_stats.preyCount;
        else ++m_stats.predatorCount;
    }
    m_stats.entityCount = m_stats.preyCount + m_stats.predatorCount;
    m_config.entityCount = m_stats.entityCount;
    if (m_config.execution.useSpatialGrid) {
        m_stats.occupiedGridCells = m_grid.getCells().size();
    }
}

void Simulation::reset() {
    m_entities.clear();
    for (std::size_t i = 0; i < m_config.preyCount; ++i) {
        m_entities.push_back({{Random::range(0.0f, m_config.width), Random::range(0.0f, m_config.height)}, {Random::range(-50.0f, 50.0f), Random::range(-50.0f, 50.0f)}, AgentType::Prey, 1.0f, true});
    }
    for (std::size_t i = 0; i < m_config.predatorCount; ++i) {
        m_entities.push_back({{Random::range(0.0f, m_config.width), Random::range(0.0f, m_config.height)}, {Random::range(-40.0f, 40.0f), Random::range(-40.0f, 40.0f)}, AgentType::Predator, 1.0f, true});
    }
    updateStats();
}

void Simulation::spawn(const Vec2& position) {
    simfw::simulation::spawnBrush(m_stats.preyCount, m_config.maxPreyCount, m_config.spawnCount, [this, position]() {
        m_entities.push_back({position + simfw::simulation::randomDiscOffset(m_config.brushRadius), {Random::range(-50.0f, 50.0f), Random::range(-50.0f, 50.0f)}, AgentType::Prey, 1.0f, true});
    });
    updateStats();
}

void Simulation::update(float dt) {
    m_previousBoids = m_entities;
    m_stats.catches = 0;

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        if (!m_previousBoids[i].alive) continue;
        const Boid& b = m_previousBoids[i];
        Vec2 accel{};

        if (b.type == AgentType::Prey) {
            Vec2 align{}, cohesion{}, separation{}, flee{};
            int alignCount = 0, cohesionCount = 0, separationCount = 0, fleeCount = 0;

            for (std::size_t j = 0; j < m_previousBoids.size(); ++j) {
                if (i == j || !m_previousBoids[j].alive) continue;
                const Boid& other = m_previousBoids[j];
                const Vec2 delta = other.position - b.position;
                const float d = delta.length();
                if (other.type == AgentType::Prey) {
                    if (d < m_config.preyPerceptionRadius) { align += other.velocity; cohesion += other.position; ++alignCount; ++cohesionCount; }
                    if (d < m_config.preySeparationRadius && d > 0.001f) { separation -= delta / d; ++separationCount; }
                } else if (d < m_config.predatorPerceptionRadius) {
                    if (d > 0.001f) flee -= delta / d;
                    ++fleeCount;
                }
            }

            if (alignCount > 0) accel += steerTowards(align / static_cast<float>(alignCount), b.velocity, m_config.preyMaxSpeed) * m_config.alignmentWeight;
            if (cohesionCount > 0) accel += steerTowards((cohesion / static_cast<float>(cohesionCount)) - b.position, b.velocity, m_config.preyMaxSpeed) * m_config.cohesionWeight;
            if (separationCount > 0) accel += steerTowards(separation / static_cast<float>(separationCount), b.velocity, m_config.preyMaxSpeed) * m_config.separationWeight;
            if (fleeCount > 0) accel += steerTowards(flee / static_cast<float>(fleeCount), b.velocity, m_config.preyMaxSpeed) * m_config.fleeWeight;
            accel = limitLength(accel, m_config.preyMaxForce);

            m_entities[i].velocity = limitLength(b.velocity + accel * dt, m_config.preyMaxSpeed);
            m_entities[i].position = wrap(b.position + m_entities[i].velocity * dt, m_config);
        } else {
            float closest = m_config.predatorPerceptionRadius;
            std::size_t target = m_previousBoids.size();
            for (std::size_t j = 0; j < m_previousBoids.size(); ++j) {
                if (!m_previousBoids[j].alive || m_previousBoids[j].type != AgentType::Prey) continue;
                const float d = (m_previousBoids[j].position - b.position).length();
                if (d < closest) { closest = d; target = j; }
            }

            if (target < m_previousBoids.size()) {
                accel = steerTowards(m_previousBoids[target].position - b.position, b.velocity, m_config.predatorMaxSpeed) * m_config.chaseWeight;
                if (closest < m_config.catchRadius) {
                    m_entities[target].alive = false;
                    m_entities[i].energy = std::min(1.5f, m_entities[i].energy + m_config.predatorEnergyGainOnCatch);
                    ++m_stats.catches;
                }
            }

            accel = limitLength(accel, m_config.predatorMaxForce);
            m_entities[i].velocity = limitLength(b.velocity + accel * dt, m_config.predatorMaxSpeed);
            m_entities[i].position = wrap(b.position + m_entities[i].velocity * dt, m_config);
            m_entities[i].energy -= m_config.predatorDrainPerSecond * dt;
            if (m_entities[i].energy <= 0.0f) {
                m_entities[i].energy = 1.0f;
                m_entities[i].position = {Random::range(0.0f, m_config.width), Random::range(0.0f, m_config.height)};
            }
        }
    }

    m_entities.erase(std::remove_if(m_entities.begin(), m_entities.end(), [](const Boid& b) { return !b.alive; }), m_entities.end());
    if (m_config.execution.useSpatialGrid) {
        m_grid.clear();
        for (std::size_t i = 0; i < m_entities.size(); ++i) {
            m_grid.insert(i, m_entities[i].position);
        }
    }
    updateStats();
}

} // namespace predator_prey_cpu
