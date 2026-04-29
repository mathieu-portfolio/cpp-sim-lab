#include "Simulation.hpp"
#include "BubblePhysics.hpp"
#include "thread_pool.hpp"

#include <simulation/CollisionPairs.hpp>
#include <simulation/ParallelUpdate.hpp>
#include <simulation/SpatialQuery.hpp>

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>
#include <thread>

namespace bubbles_cpu {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;

Vec2 bubblePosition(const Bubble& bubble) {
    return bubble.position;
}

std::size_t hardwareWorkerCount() {
    const unsigned int hardwareThreads = std::thread::hardware_concurrency();
    return hardwareThreads == 0 ? 1 : static_cast<std::size_t>(hardwareThreads);
}
} // namespace

Simulation::Simulation(SimulationConfig config)
    : Base(config),
      m_grid(m_config.gridCellSize),
      m_threadPool(std::make_unique<ThreadPool>(hardwareWorkerCount())) {
    m_config.cellSize = m_config.gridCellSize;
    m_config.entityCount = m_config.maxBubbleCount;
    m_entities.reserve(m_config.maxBubbleCount);
    m_stats.maxBubbleCount = m_config.maxBubbleCount;
    updateStatsCount();
}

Simulation::~Simulation() = default;

void Simulation::updateStatsCount() {
    m_stats.bubbleCount = m_entities.size();
    m_stats.entityCount = m_entities.size();
    m_stats.maxBubbleCount = m_config.maxBubbleCount;
}

void Simulation::integrateBubbleRange(std::size_t beginIndex, std::size_t endIndex, float dt) {
    for (std::size_t i = beginIndex; i < endIndex; ++i) {
        Bubble& bubble = m_entities[i];
        bubble.age += dt;
        bubble.stress *= 0.92f;

        const float pressureError = bubble.restRadius - bubble.radius;
        bubble.radius += pressureError * m_config.pressureStrength * dt * 0.2f;
        bubble.radius = std::max(2.0f, bubble.radius);

        bubble.velocity.y -= m_config.buoyancy * dt;
        bubble.velocity *= m_config.damping;
        bubble.position += bubble.velocity * dt;
        solveBubbleBounds(bubble, m_config);
    }
}

void Simulation::integrateBubbles(float dt) {
    simfw::runParallelUpdate<ThreadPool, BubbleUpdateScratch, SimulationStats>(
        m_threadPool.get(),
        m_entities.size(),
        MinItemsPerParallelTask,
        m_config.execution.useParallelUpdate,
        [this, dt](std::size_t beginIndex, std::size_t endIndex, BubbleUpdateScratch&, SimulationStats&) {
            integrateBubbleRange(beginIndex, endIndex, dt);
        },
        [](const SimulationStats&) {}
    );
}

void Simulation::buildSpatialIndex() {
    m_grid.setCellSize(m_config.gridCellSize);
    if (m_config.execution.useSpatialGrid) {
        m_grid.build(m_entities, bubblePosition);
    } else {
        m_grid.clear();
    }
}

void Simulation::resolveInteractions(float dt) {
    const float queryRadius = m_config.baseRadius * 4.0f;
    simfw::simulation::forEachUniqueCandidatePair<int>(
        m_entities.size(),
        [this, queryRadius](std::size_t index, std::vector<int>& candidates) {
            simfw::simulation::collectCandidates(
                m_grid,
                m_entities[index].position,
                queryRadius,
                simfw::simulation::makeSpatialQueryOptions<int>(m_config.execution.useSpatialGrid, m_entities.size()),
                candidates
            );
        },
        [this, dt](int firstIndex, int secondIndex) {
            ++m_stats.interactionChecks;
            Bubble& first = m_entities[static_cast<std::size_t>(firstIndex)];
            Bubble& second = m_entities[static_cast<std::size_t>(secondIndex)];
            applySurfaceTension(first, second, m_config, dt);
            if (resolveBubbleCollision(first, second, m_config, dt)) {
                ++m_stats.collisionsResolved;
                solveBubbleBounds(first, m_config);
                solveBubbleBounds(second, m_config);
            }
        }
    );
}

void Simulation::mergeCloseBubbles() {
    if (!m_config.enableMerge || m_entities.size() < 2) {
        return;
    }

    std::vector<Bubble> merged;
    std::vector<bool> consumed(m_entities.size(), false);

    for (std::size_t i = 0; i < m_entities.size(); ++i) {
        if (consumed[i]) {
            continue;
        }

        Bubble current = m_entities[i];
        for (std::size_t j = i + 1; j < m_entities.size(); ++j) {
            if (consumed[j]) {
                continue;
            }

            const Bubble& other = m_entities[j];
            const float mergeDistance = (current.radius + other.radius) * m_config.mergeDistanceFactor;
            if ((other.position - current.position).length() <= mergeDistance) {
                current = mergeBubbles(current, other);
                consumed[j] = true;
                ++m_stats.mergedCount;
            }
        }

        merged.push_back(current);
    }

    m_entities.swap(merged);
}

void Simulation::applyBursting() {
    if (!m_config.enableBurst) {
        return;
    }

    const auto oldSize = m_entities.size();
    m_entities.erase(
        std::remove_if(m_entities.begin(), m_entities.end(), [this](const Bubble& bubble) {
            return bubble.age > m_config.burstAgeThreshold || bubble.stress > m_config.burstStressThreshold;
        }),
        m_entities.end()
    );
    m_stats.burstCount += oldSize - m_entities.size();
}

void Simulation::update(float dt) {
    m_stats = {};
    m_stats.maxBubbleCount = m_config.maxBubbleCount;
    dt = std::min(dt, 1.0f / 30.0f);

    integrateBubbles(dt);
    buildSpatialIndex();
    resolveInteractions(dt);
    mergeCloseBubbles();
    applyBursting();

    for (auto& bubble : m_entities) {
        solveBubbleBounds(bubble, m_config);
    }

    updateStatsCount();
}

void Simulation::spawn(const Vec2& position) {
    for (int i = 0; i < m_config.spawnCount && m_entities.size() < m_config.maxBubbleCount; ++i) {
        const float radius = Random::range(m_config.baseRadius * 0.75f, m_config.baseRadius * 1.35f);
        m_entities.push_back({
            position + Vec2{Random::range(-m_config.spawnJitter, m_config.spawnJitter), Random::range(-m_config.spawnJitter, m_config.spawnJitter)},
            Vec2{Random::range(-20.0f, 20.0f), Random::range(-40.0f, 20.0f)},
            radius,
            radius,
            0.0f,
            0.0f
        });
    }
    updateStatsCount();
}

void Simulation::clear() {
    m_entities.clear();
    m_grid.clear();
    m_stats = {};
    m_stats.maxBubbleCount = m_config.maxBubbleCount;
    updateStatsCount();
}

void Simulation::reset() {
    clear();
    for (int i = 0; i < 80 && m_entities.size() < m_config.maxBubbleCount; ++i) {
        spawn({Random::range(80.0f, m_config.width - 80.0f), Random::range(180.0f, m_config.height - 100.0f)});
    }
    updateStatsCount();
}

} // namespace bubbles_cpu
