#pragma once

#include "Agent.hpp"
#include "Obstacle.hpp"
#include "Simulation.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace agents_cpu::steering {

struct CandidateLists {
    std::span<const std::size_t> agents;
    std::span<const std::size_t> obstacles;
};

struct BehaviorContext {
    const SimulationConfig& config;
    const std::vector<Agent>& agents;
    const std::vector<Obstacle>& obstacles;
    CandidateLists candidates;
    SimulationStats& stats;
};

using BehaviorFn = Vec2 (*)(std::size_t agentIndex, BehaviorContext& context);

enum class ForceScale {
    Unit,
    MaxForce
};

struct WeightedBehavior {
    BehaviorFn compute;
    float SimulationConfig::* weight;
    ForceScale scale = ForceScale::Unit;
};

std::span<const WeightedBehavior> defaultBehaviors();

Vec2 limitLength(Vec2 value, float maxLength);

Vec2 computeAcceleration(std::size_t agentIndex, BehaviorContext& context);

Vec2 seek(std::size_t agentIndex, BehaviorContext& context);
Vec2 separate(std::size_t agentIndex, BehaviorContext& context);
Vec2 avoidObstacles(std::size_t agentIndex, BehaviorContext& context);

void resolveObstacleOverlap(
    Agent& agent,
    const std::vector<Obstacle>& obstacles,
    std::span<const std::size_t> obstacleCandidates,
    SimulationStats& stats
);

} // namespace agents_cpu::steering
