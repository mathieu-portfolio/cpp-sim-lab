#include "BoidBehavior.hpp"
#include "Simulation.hpp"

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
} // namespace

std::span<const WeightedBoidBehavior> defaultBoidBehaviors() {
    static constexpr std::array<WeightedBoidBehavior, 3> Behaviors{{
        WeightedBoidBehavior{
            BoidBehaviorType::Alignment,
            BoidBehaviorWeight::Alignment
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Cohesion,
            BoidBehaviorWeight::Cohesion
        },
        WeightedBoidBehavior{
            BoidBehaviorType::Separation,
            BoidBehaviorWeight::Separation
        }
    }};

    return Behaviors;
}

Vec2 computeAcceleration(
    std::size_t boidIndex,
    BoidBehaviorContext& context,
    std::span<const WeightedBoidBehavior> behaviors
) {
    Vec2 acceleration{};

    for (const WeightedBoidBehavior& behavior : behaviors) {
        if (!behavior.enabled) {
            continue;
        }

        acceleration += computeBehavior(
            behavior.type,
            boidIndex,
            context
        ) * behaviorWeight(context.config, behavior);
    }

    return limitLength(acceleration, context.config.maxForce);
}

Vec2 computeBehavior(
    BoidBehaviorType behaviorType,
    std::size_t boidIndex,
    BoidBehaviorContext& context
) {
    switch (behaviorType) {
    case BoidBehaviorType::Alignment:
        return computeAlignment(boidIndex, context);
    case BoidBehaviorType::Cohesion:
        return computeCohesion(boidIndex, context);
    case BoidBehaviorType::Separation:
        return computeSeparation(boidIndex, context);
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
