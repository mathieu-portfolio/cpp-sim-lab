#include "BoidBehavior.hpp"
#include "Simulation.hpp"

#include <simulation/WeightedBehaviorPipeline.hpp>

#include <array>

namespace boids {
namespace {
float behaviorWeight(
    const SimulationConfig& config,
    const WeightedBoidBehavior& behavior
) {
    switch (behavior.weight) {
    case BoidBehaviorWeight::Alignment:
        return config.alignmentWeight;
    case BoidBehaviorWeight::Cohesion:
        return config.cohesionWeight;
    case BoidBehaviorWeight::Separation:
        return config.separationWeight;
    case BoidBehaviorWeight::Wander:
        return config.wanderWeight;
    case BoidBehaviorWeight::Fixed:
        return behavior.fixedWeight;
    }

    return behavior.fixedWeight;
}

float scaleFactor(const WeightedBoidBehavior& behavior) {
    switch (behavior.scale) {
    case ForceScale::Unit:
        return 1.0f;
    }

    return 1.0f;
}

BoidBehaviorFn behaviorFunction(BoidBehaviorType type) {
    switch (type) {
    case BoidBehaviorType::Alignment:
        return computeAlignmentBehavior;
    case BoidBehaviorType::Cohesion:
        return computeCohesionBehavior;
    case BoidBehaviorType::Separation:
        return computeSeparationBehavior;
    case BoidBehaviorType::Wander:
        return computeWanderBehavior;
    }

    return nullptr;
}

const char* behaviorName(BoidBehaviorType type) {
    switch (type) {
    case BoidBehaviorType::Alignment:
        return "alignment";
    case BoidBehaviorType::Cohesion:
        return "cohesion";
    case BoidBehaviorType::Separation:
        return "separation";
    case BoidBehaviorType::Wander:
        return "wander";
    }

    return "unknown";
}
} // namespace

std::span<const WeightedBoidBehavior> defaultBoidBehaviors() {
    static constexpr std::array<WeightedBoidBehavior, 4> Behaviors{{
        WeightedBoidBehavior{
            BoidBehaviorType::Alignment,
            computeAlignmentBehavior,
            BoidBehaviorWeight::Alignment,
            ForceScale::Unit,
            1.0f,
            true,
            "alignment"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Cohesion,
            computeCohesionBehavior,
            BoidBehaviorWeight::Cohesion,
            ForceScale::Unit,
            1.0f,
            true,
            "cohesion"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Separation,
            computeSeparationBehavior,
            BoidBehaviorWeight::Separation,
            ForceScale::Unit,
            1.0f,
            true,
            "separation"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Wander,
            computeWanderBehavior,
            BoidBehaviorWeight::Wander,
            ForceScale::Unit,
            1.0f,
            true,
            "wander"
        }
    }};

    return Behaviors;
}

std::vector<WeightedBoidBehavior> makeDefaultBoidBehaviors() {
    const auto defaults = defaultBoidBehaviors();
    return std::vector<WeightedBoidBehavior>{defaults.begin(), defaults.end()};
}

WeightedBoidBehavior makeBoidBehavior(
    BoidBehaviorType type,
    BoidBehaviorWeight weight,
    float fixedWeight,
    bool enabled
) {
    return WeightedBoidBehavior{
        type,
        behaviorFunction(type),
        weight,
        ForceScale::Unit,
        fixedWeight,
        enabled,
        behaviorName(type)
    };
}

Vec2 computeAcceleration(
    std::size_t boidIndex,
    BoidBehaviorContext& context,
    std::span<const WeightedBoidBehavior> behaviors
) {
    const Vec2 acceleration = simfw::simulation::computeWeightedBehaviors<
        WeightedBoidBehavior,
        BoidBehaviorContext,
        Vec2
    >(
        boidIndex,
        behaviors,
        context,
        [](const WeightedBoidBehavior& behavior, const BoidBehaviorContext& ctx) {
            return behaviorWeight(ctx.config, behavior);
        },
        simfw::simulation::AlwaysApplyBehavior{},
        [](const WeightedBoidBehavior& behavior, const BoidBehaviorContext&) {
            return scaleFactor(behavior);
        }
    );

    return limitLength(acceleration, context.config.maxForce);
}

Vec2 computeBehavior(
    BoidBehaviorType behaviorType,
    std::size_t boidIndex,
    BoidBehaviorContext& context
) {
    if (const BoidBehaviorFn fn = behaviorFunction(behaviorType)) {
        return fn(boidIndex, context);
    }

    return {};
}

Vec2 computeAlignmentBehavior(std::size_t boidIndex, BoidBehaviorContext& context) {
    return computeAlignment(
        boidIndex,
        context.boids,
        context.candidates.perception,
        context.config.maxSpeed
    );
}

Vec2 computeCohesionBehavior(std::size_t boidIndex, BoidBehaviorContext& context) {
    return computeCohesion(
        boidIndex,
        context.boids,
        context.candidates.perception,
        context.config.maxSpeed
    );
}

Vec2 computeSeparationBehavior(std::size_t boidIndex, BoidBehaviorContext& context) {
    return computeSeparation(
        boidIndex,
        context.boids,
        context.candidates.separation,
        context.config.maxSpeed
    );
}

Vec2 computeWanderBehavior(std::size_t boidIndex, BoidBehaviorContext& context) {
    return computeWander(
        boidIndex,
        context.boids,
        context.config.maxSpeed,
        context.config.wanderJitter
    );
}

} // namespace boids
