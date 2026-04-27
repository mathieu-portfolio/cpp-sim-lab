#include "Simulation.hpp"
#include "BoidBehavior.hpp"

#include <algorithm>
#include <random/Random.hpp>

namespace {
void addNeighborsFromCandidateList(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& candidates,
    float perceptionRadius,
    float separationRadius,
    std::vector<std::size_t>& perceptionNeighbors,
    std::vector<std::size_t>& separationNeighbors,
    SimulationStats& stats
) {
    perceptionNeighbors.clear();
    separationNeighbors.clear();

    for (std::size_t candidateIndex : candidates) {
        if (candidateIndex == boidIndex) {
            continue;
        }

        ++stats.neighborChecks;

        const float distance =
            (boids[candidateIndex].position - boids[boidIndex].position).length();

        if (distance < perceptionRadius) {
            perceptionNeighbors.push_back(candidateIndex);
        }

        if (distance < separationRadius) {
            separationNeighbors.push_back(candidateIndex);
        }
    }
}

void addNeighborsNaive(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    float perceptionRadius,
    float separationRadius,
    std::vector<std::size_t>& perceptionNeighbors,
    std::vector<std::size_t>& separationNeighbors,
    SimulationStats& stats
) {
    perceptionNeighbors.clear();
    separationNeighbors.clear();

    for (std::size_t i = 0; i < boids.size(); ++i) {
        if (i == boidIndex) {
            continue;
        }

        ++stats.neighborChecks;

        const float distance =
            (boids[i].position - boids[boidIndex].position).length();

        if (distance < perceptionRadius) {
            perceptionNeighbors.push_back(i);
        }

        if (distance < separationRadius) {
            separationNeighbors.push_back(i);
        }
    }
}
}

Simulation::Simulation(SimulationConfig config)
    : m_config(config),
      m_grid(config.gridCellSize) {
    normalizeConfigCounts();
    reset();
}

void Simulation::normalizeConfigCounts() {
    constexpr std::size_t defaultCount = SimulationConfig::DefaultBoidCount;

    if (m_config.boidCount == defaultCount && m_config.entityCount != defaultCount) {
        m_config.boidCount = m_config.entityCount;
    } else {
        m_config.entityCount = m_config.boidCount;
    }
}

void Simulation::updateStatsCount() {
    m_stats.boidCount = m_boids.size();
    m_stats.entityCount = m_boids.size();
}

void Simulation::reset() {
    normalizeConfigCounts();

    m_boids.clear();
    m_boids.reserve(m_config.boidCount);

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

    m_stats = {};
    updateStatsCount();
}

void Simulation::update(float dt) {
    normalizeConfigCounts();

    m_stats = {};
    updateStatsCount();

    m_grid.setCellSize(m_config.gridCellSize);

    if (m_config.useSpatialGrid) {
        m_grid.build(m_boids);
        m_stats.occupiedGridCells = m_grid.getCells().size();
    } else {
        m_grid.clear();
    }

    std::vector<std::size_t> candidates;
    std::vector<std::size_t> perceptionNeighbors;
    std::vector<std::size_t> separationNeighbors;

    const float queryRadius =
        std::max(m_config.perceptionRadius, m_config.separationRadius);

    for (std::size_t i = 0; i < m_boids.size(); ++i) {
        if (m_config.useSpatialGrid) {
            m_grid.queryNeighbors(m_boids[i].position, queryRadius, candidates);
            m_stats.neighborCandidates += candidates.size();

            addNeighborsFromCandidateList(
                i,
                m_boids,
                candidates,
                m_config.perceptionRadius,
                m_config.separationRadius,
                perceptionNeighbors,
                separationNeighbors,
                m_stats
            );
        } else {
            addNeighborsNaive(
                i,
                m_boids,
                m_config.perceptionRadius,
                m_config.separationRadius,
                perceptionNeighbors,
                separationNeighbors,
                m_stats
            );
        }

        Vec2 align = computeAlignment(
            i,
            m_boids,
            perceptionNeighbors,
            m_config.maxSpeed
        );

        Vec2 coh = computeCohesion(
            i,
            m_boids,
            perceptionNeighbors,
            m_config.maxSpeed
        );

        Vec2 sep = computeSeparation(
            i,
            m_boids,
            separationNeighbors,
            m_config.maxSpeed
        );

        Vec2 force =
            align * m_config.alignmentWeight +
            coh * m_config.cohesionWeight +
            sep * m_config.separationWeight;

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
