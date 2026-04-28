#include "BoidBehavior.hpp"
#include "Simulation.hpp"

#include <simulation/WeightedBehaviorPipeline.hpp>

#include <array>

namespace boids_cpu {
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
    case BoidBehaviorWeight::Fixed:
        return behavior.fixedWeight;
    }

    return behavior.fixedWeight;
}

BoidBehaviorFn behaviorFunction(BoidBehaviorType type) {
    switch (type) {
    case BoidBehaviorType::Alignment:
        return computeAlignmentBehavior;
    case BoidBehaviorType::Cohesion:
        return computeCohesionBehavior;
    case BoidBehaviorType::Separation:
        return computeSeparationBehavior;
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
    }

    return "unknown";
}
} // namespace

std::span<const WeightedBoidBehavior> defaultBoidBehaviors() {
    static constexpr std::array<WeightedBoidBehavior, 3> Behaviors{{
        WeightedBoidBehavior{
            BoidBehaviorType::Alignment,
            computeAlignmentBehavior,
            BoidBehaviorWeight::Alignment,
            1.0f,
            true,
            "alignment"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Cohesion,
            computeCohesionBehavior,
            BoidBehaviorWeight::Cohesion,
            1.0f,
            true,
            "cohesion"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Separation,
            computeSeparationBehavior,
            BoidBehaviorWeight::Separation,
            1.0f,
            true,
            "separation"
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

} // namespace boids_cpu
