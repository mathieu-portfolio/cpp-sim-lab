#pragma once

#include "Bubble.hpp"

#include <simulation/SimulationBase.hpp>
#include <simulation/SimulationExecutionConfig.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

class ThreadPool;

namespace bubbles {

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;

    float buoyancy = 300.0f;
    float damping = 0.995f;
    float pressureStrength = 30.0f;
    float surfaceTension = 40.0f;
    float collisionStiffness = 55.0f;
    float wallBounce = 0.2f;

    float baseRadius = 8.0f;
    float spawnJitter = 16.0f;
    int spawnCount = 4;
    float brushRadius = 16.0f;
    std::size_t maxBubbleCount = 1200;

    bool enableMerge = true;
    float mergeDistanceFactor = 0.55f;

    bool enableBurst = false;
    float burstAgeThreshold = 30.0f;
    float burstStressThreshold = 3.0f;

    float gridCellSize = 20.0f;

    float cellSize = 20.0f;
    simfw::simulation::SimulationExecutionConfig execution{};
    std::size_t entityCount = 0;
};

struct SimulationStats {
    std::size_t bubbleCount = 0;
    std::size_t maxBubbleCount = 0;
    std::size_t interactionChecks = 0;
    std::size_t collisionsResolved = 0;
    std::size_t mergedCount = 0;
    std::size_t burstCount = 0;
    std::size_t entityCount = 0;
};

class Simulation : public simfw::SimulationBase<SimulationConfig, SimulationStats, Bubble> {
public:
    using Base = simfw::SimulationBase<SimulationConfig, SimulationStats, Bubble>;
    using Grid = simfw::SpatialHashGrid<int>;

    explicit Simulation(SimulationConfig config = {});
    ~Simulation();

    void update(float dt);
    void reset();
    void spawn(const Vec2& position);
    void clear();

    const std::vector<Bubble>& getBubbles() const { return m_entities; }
    const Grid& getGrid() const { return m_grid; }

private:
    struct BubbleUpdateScratch {};

    Grid m_grid;
    std::unique_ptr<ThreadPool> m_threadPool;

    void updateStatsCount();
    void integrateBubbleRange(std::size_t beginIndex, std::size_t endIndex, float dt);
    void integrateBubbles(float dt);
    void buildSpatialIndex();
    void resolveInteractions(float dt);
    void mergeCloseBubbles();
    void applyBursting();
};

} // namespace bubbles
