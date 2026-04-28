#include "BoidBehavior.hpp"
#include "Simulation.hpp"

#include <simulation/WeightedBehaviorPipeline.hpp>

#include <array>

namespace boids_cpu {
namespace {
constexpr float Epsilon = 0.0001f;

Vec2 steerToward(Vec2 desiredDirection, Vec2 currentVelocity, float maxSpeed) {
    if (desiredDirection.length() <= Epsilon) {
        return {};
    }

    const Vec2 desiredVelocity = desiredDirection.normalized() * maxSpeed;
    return desiredVelocity - currentVelocity;
}

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
        return computeAlignment;
    case BoidBehaviorType::Cohesion:
        return computeCohesion;
    case BoidBehaviorType::Separation:
        return computeSeparation;
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
            computeAlignment,
            BoidBehaviorWeight::Alignment,
            1.0f,
            true,
            "alignment"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Cohesion,
            computeCohesion,
            BoidBehaviorWeight::Cohesion,
            1.0f,
            true,
            "cohesion"
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Separation,
            computeSeparation,
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

Vec2 computeAlignment(std::size_t boidIndex, BoidBehaviorContext& context) {
    const Boid& boid = context.boids[boidIndex];

    Vec2 averageVelocity{};
    int neighborCount = 0;

    for (std::size_t neighborIndex : context.candidates.perception) {
        if (neighborIndex == boidIndex) {
            continue;
        }

        averageVelocity += context.boids[neighborIndex].velocity;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    averageVelocity *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(averageVelocity, boid.velocity, context.config.maxSpeed);
}

Vec2 computeCohesion(std::size_t boidIndex, BoidBehaviorContext& context) {
    const Boid& boid = context.boids[boidIndex];

    Vec2 center{};
    int neighborCount = 0;

    for (std::size_t neighborIndex : context.candidates.perception) {
        if (neighborIndex == boidIndex) {
            continue;
        }

        center += context.boids[neighborIndex].position;
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    center *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(center - boid.position, boid.velocity, context.config.maxSpeed);
}

Vec2 computeSeparation(std::size_t boidIndex, BoidBehaviorContext& context) {
    const Boid& boid = context.boids[boidIndex];

    Vec2 away{};
    int neighborCount = 0;

    for (std::size_t neighborIndex : context.candidates.separation) {
        if (neighborIndex == boidIndex) {
            continue;
        }

        const Vec2 offset = boid.position - context.boids[neighborIndex].position;
        const float distance = offset.length();

        if (distance <= Epsilon) {
            continue;
        }

        away += offset.normalized() * (1.0f / distance);
        ++neighborCount;
    }

    if (neighborCount == 0) {
        return {};
    }

    away *= 1.0f / static_cast<float>(neighborCount);
    return steerToward(away, boid.velocity, context.config.maxSpeed);
}

Vec2 limitLength(Vec2 value, float maxLength) {
    const float length = value.length();

    if (length > maxLength && length > 0.0f) {
        return value * (maxLength / length);
    }

    return value;
}

Vec2 wrapPosition(Vec2 position, float width, float height) {
    if (position.x < 0.0f) {
        position.x += width;
    }

    if (position.x > width) {
        position.x -= width;
    }

    if (position.y < 0.0f) {
        position.y += height;
    }

    if (position.y > height) {
        position.y -= height;
    }

    return position;
}

} // namespace boids_cpu
