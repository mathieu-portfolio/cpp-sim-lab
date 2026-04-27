#include "Simulation.hpp"
#include "BoidBehavior.hpp"
#include <random/Random.hpp>

Simulation::Simulation(SimulationConfig config)
    : m_config(config) {
    reset();
}

void Simulation::reset() {
    m_boids.clear();

    for (std::size_t i = 0; i < m_config.boidCount; ++i) {
        m_boids.push_back({
            Vec2{
                Random::range(0.0f, m_config.width),
                Random::range(0.0f, m_config.height)
            },
            Vec2{
                Random::range(-50.0f, 50.0f),
                Random::range(-50.0f, 50.0f)
            }
        });
    }

    m_stats.boidCount = m_boids.size();
    m_stats.neighborChecks = 0;
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.boidCount = m_boids.size();

    const std::size_t boidCount = m_boids.size();

    if (boidCount > 0) {
        m_stats.neighborChecks = boidCount * (boidCount - 1) * 3;
    }

    std::vector<std::size_t> neighbors;

    for (std::size_t i = 0; i < m_boids.size(); ++i) {
        neighbors.clear();

        for (std::size_t j = 0; j < m_boids.size(); ++j) {
            if (i == j) continue;

            Vec2 offset = m_boids[j].position - m_boids[i].position;
            float dist = offset.length();

            if (dist < m_config.perceptionRadius) {
                neighbors.push_back(j);
            }

            // stats
            ++m_stats.neighborChecks;
        }

        Vec2 align = computeAlignment(i, m_boids, neighbors, m_config.maxSpeed);
        Vec2 coh   = computeCohesion(i, m_boids, neighbors, m_config.maxSpeed);

        // reuse neighbors for separation or recompute with smaller radius
        std::vector<std::size_t> sepNeighbors;

        for (std::size_t idx : neighbors) {
            float dist = (m_boids[idx].position - m_boids[i].position).length();
            if (dist < m_config.separationRadius) {
                sepNeighbors.push_back(idx);
            }
        }

        Vec2 sep = computeSeparation(i, m_boids, sepNeighbors, m_config.maxSpeed);

        Vec2 force =
            align * m_config.alignmentWeight +
            coh   * m_config.cohesionWeight +
            sep   * m_config.separationWeight;

        force = limitLength(force, m_config.maxForce);

        m_boids[i].velocity += force * dt;
        m_boids[i].velocity = limitLength(m_boids[i].velocity, m_config.maxSpeed);
        m_boids[i].position += m_boids[i].velocity * dt;

        m_boids[i].position = wrapPosition(
            m_boids[i].position,
            m_config.width,
            m_config.height
        );
    }
}
