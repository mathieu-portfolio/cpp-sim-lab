#pragma once

#include "BoidSteering.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace boids_cpu {

struct SimulationConfig;
struct SimulationStats;
struct BoidBehaviorContext;

using BoidBehaviorFn = Vec2 (*)(std::size_t boidIndex, BoidBehaviorContext& context);

enum class BoidBehaviorType {
    Alignment,
    Cohesion,
    Separation,
    Wander
};

enum class BoidBehaviorWeight {
    Alignment,
    Cohesion,
    Separation,
    Wander,
    Fixed
};

enum class ForceScale {
    Unit
};

struct WeightedBoidBehavior {
    BoidBehaviorType type = BoidBehaviorType::Alignment;
    BoidBehaviorFn compute = nullptr;
    BoidBehaviorWeight weight = BoidBehaviorWeight::Fixed;
    ForceScale scale = ForceScale::Unit;
    float fixedWeight = 1.0f;
    bool enabled = true;
    const char* name = "";
};

struct BoidCandidateLists {
    std::span<const std::size_t> perception;
    std::span<const std::size_t> separation;
};

struct BoidBehaviorContext {
    const SimulationConfig& config;
    const std::vector<Boid>& boids;
    BoidCandidateLists candidates;
    SimulationStats& stats;
};

std::span<const WeightedBoidBehavior> defaultBoidBehaviors();
std::vector<WeightedBoidBehavior> makeDefaultBoidBehaviors();

WeightedBoidBehavior makeBoidBehavior(
    BoidBehaviorType type,
    BoidBehaviorWeight weight,
    float fixedWeight = 1.0f,
    bool enabled = true
);

Vec2 computeAcceleration(
    std::size_t boidIndex,
    BoidBehaviorContext& context,
    std::span<const WeightedBoidBehavior> behaviors
);

Vec2 computeBehavior(
    BoidBehaviorType behaviorType,
    std::size_t boidIndex,
    BoidBehaviorContext& context
);

Vec2 computeAlignmentBehavior(std::size_t boidIndex, BoidBehaviorContext& context);
Vec2 computeCohesionBehavior(std::size_t boidIndex, BoidBehaviorContext& context);
Vec2 computeSeparationBehavior(std::size_t boidIndex, BoidBehaviorContext& context);
Vec2 computeWanderBehavior(std::size_t boidIndex, BoidBehaviorContext& context);

} // namespace boids_cpu
