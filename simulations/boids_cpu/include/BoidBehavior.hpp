#pragma once

#include "Boid.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace boids_cpu {

struct SimulationConfig;
struct SimulationStats;

enum class BoidBehaviorType {
    Alignment,
    Cohesion,
    Separation
};

enum class BoidBehaviorWeight {
    Alignment,
    Cohesion,
    Separation,
    Fixed
};

struct WeightedBoidBehavior {
    BoidBehaviorType type = BoidBehaviorType::Alignment;
    BoidBehaviorWeight weight = BoidBehaviorWeight::Fixed;
    float fixedWeight = 1.0f;
    bool enabled = true;
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

Vec2 computeAlignment(std::size_t boidIndex, BoidBehaviorContext& context);
Vec2 computeCohesion(std::size_t boidIndex, BoidBehaviorContext& context);
Vec2 computeSeparation(std::size_t boidIndex, BoidBehaviorContext& context);

Vec2 limitLength(Vec2 value, float maxLength);
Vec2 wrapPosition(Vec2 position, float width, float height);

} // namespace boids_cpu
