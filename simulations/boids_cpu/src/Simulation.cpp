#include "Simulation.hpp"
#include "BoidBehavior.hpp"

#include <algorithm>
#include <random/Random.hpp>

Simulation::Simulation(SimulationConfig config)
    : m_config(config),
      m_grid(config.gridCellSize) {
    reset();
}

void Simulation::reset() {
    m_boids.clear();
    m_boids.reserve(m_config.entityCount);

    for (std::size_t i = 0; i < m_config.entityCount; ++i) {
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
    m_stats.entityCount = m_boids.size();
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.entityCount = m_boids.size();

    m_grid.setCellSize(m_config.gridCellSize);

    if (m_config.useSpatialGrid) {
        m_grid.build(m_boids);
        m_stats.occupiedGridCells = m_grid.getCells().size();
    } else {
        m_grid.clear();
    }

    std::vector<std::size_t> candidates;
    std::vector<std::size_t> neighbors;

    const float queryRadius = m_config.perceptionRadius;

    for (std::size_t i = 0; i < m_boids.size(); ++i) {
        if (m_config.useSpatialGrid) {
            m_grid.queryNeighbors(m_boids[i].position, queryRadius, candidates);
            m_stats.neighborCandidates += candidates.size();
        } else {
            candidates.clear();
            for (std::size_t j = 0; j < m_boids.size(); ++j) {
                if (j != i) candidates.push_back(j);
            }
        }

        for (auto idx : candidates) {
            if (idx == i) continue;
            ++m_stats.neighborChecks;
            neighbors.push_back(idx);
        }

        neighbors.clear();
    }
}
