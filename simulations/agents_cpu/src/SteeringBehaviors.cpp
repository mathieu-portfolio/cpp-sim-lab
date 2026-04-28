#include "SteeringBehaviors.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace agents_cpu::steering {
namespace {
constexpr float Epsilon = 0.0001f;

float scaleFactor(const SimulationConfig& config, ForceScale scale) {
    switch (scale) {
    case ForceScale::Unit:
        return 1.0f;
    case ForceScale::MaxForce:
        return config.maxForce;
    }

    return 1.0f;
}
} // namespace

std::span<const WeightedBehavior> defaultBehaviors() {
    static constexpr std::array<WeightedBehavior, 3> Behaviors{{
        WeightedBehavior{seek, &SimulationConfig::seekWeight, ForceScale::Unit},
        WeightedBehavior{separate, &SimulationConfig::separationWeight, ForceScale::MaxForce},
        WeightedBehavior{avoidObstacles, &SimulationConfig::obstacleAvoidanceWeight, ForceScale::MaxForce}
    }};

    return Behaviors;
}

Vec2 limitLength(Vec2 value, float maxLength) {
    const float length = value.length();

    if (length <= maxLength || length <= Epsilon) {
        return value;
    }

    return value * (maxLength / length);
}

Vec2 computeAcceleration(std::size_t agentIndex, BehaviorContext& context) {
    Vec2 acceleration{};

    for (const WeightedBehavior& behavior : defaultBehaviors()) {
        const float weight = context.config.*(behavior.weight);

        // Always evaluate behaviors, even when their weight is zero.
        // Some behaviors also collect instrumentation stats while scanning
        // candidates, and those stats should stay independent from the
        // weighted steering contribution.
        const Vec2 force = behavior.compute(agentIndex, context);

        acceleration += force *
            (weight * scaleFactor(context.config, behavior.scale));
    }

    return limitLength(acceleration, context.config.maxForce);
}

Vec2 seek(std::size_t agentIndex, BehaviorContext& context) {
    if (context.intent == AgentIntent::Idle) {
        return Vec2{};
    }

    const SimulationConfig& config = context.config;
    const Agent& agent = context.agents[agentIndex];

    const Vec2 toTarget = agent.target - agent.position;
    const float distance = toTarget.length();

    if (distance <= Epsilon) {
        return Vec2{};
    }

    float desiredSpeed = config.maxSpeed;

    if (distance < config.arrivalRadius) {
        desiredSpeed *= std::clamp(distance / config.arrivalRadius, 0.0f, 1.0f);
    }

    const Vec2 desiredVelocity = toTarget.normalized() * desiredSpeed;
    Vec2 force = limitLength(desiredVelocity - agent.velocity, config.maxForce);

    if (context.intent == AgentIntent::AvoidObstacle) {
        force *= config.avoidIntentSeekScale;
    }

    return force;
}

Vec2 separate(std::size_t agentIndex, BehaviorContext& context) {
    const SimulationConfig& config = context.config;
    const std::vector<Agent>& agents = context.agents;

    Vec2 force{};
    int neighborCount = 0;

    for (std::size_t candidateIndex : context.candidates.agents) {
        if (candidateIndex == agentIndex) {
            continue;
        }

        ++context.stats.neighborChecks;

        const Vec2 away = agents[agentIndex].position - agents[candidateIndex].position;
        const float distance = away.length();

        if (distance <= Epsilon || distance >= config.separationRadius) {
            continue;
        }

        const float strength = 1.0f - (distance / config.separationRadius);
        force += away.normalized() * strength;
        ++neighborCount;
    }

    if (neighborCount > 0) {
        force *= 1.0f / static_cast<float>(neighborCount);
    }

    return force;
}

Vec2 avoidObstacles(std::size_t agentIndex, BehaviorContext& context) {
    const SimulationConfig& config = context.config;
    const Agent& agent = context.agents[agentIndex];

    Vec2 force{};
    int obstacleCount = 0;

    for (std::size_t obstacleIndex : context.candidates.obstacles) {
        ++context.stats.obstacleChecks;

        const Obstacle& obstacle = context.obstacles[obstacleIndex];
        const Vec2 away = agent.position - obstacle.position;
        const float distance = away.length();
        const float influenceDistance = obstacle.radius + config.obstacleAvoidanceRadius;

        if (distance <= Epsilon || distance >= influenceDistance) {
            continue;
        }

        const float strength = 1.0f - (distance / influenceDistance);
        force += away.normalized() * strength;
        ++obstacleCount;
    }

    if (obstacleCount > 0) {
        force *= 1.0f / static_cast<float>(obstacleCount);
    }

    return force;
}

void resolveObstacleOverlap(
    Agent& agent,
    const std::vector<Obstacle>& obstacles,
    std::span<const std::size_t> obstacleCandidates,
    SimulationStats& stats
) {
    for (std::size_t obstacleIndex : obstacleCandidates) {
        ++stats.obstacleOverlapChecks;

        const Obstacle& obstacle = obstacles[obstacleIndex];
        Vec2 away = agent.position - obstacle.position;
        float distance = away.length();

        const float minDistance = obstacle.radius + agent.radius;

        if (distance <= Epsilon || distance >= minDistance) {
            continue;
        }

        Vec2 normal = away * (1.0f / distance);
        agent.position = obstacle.position + normal * minDistance;

        const float velocityIntoObstacle = Vec2::dot(agent.velocity, normal);

        if (velocityIntoObstacle < 0.0f) {
            agent.velocity -= normal * velocityIntoObstacle;
        }
    }
}

} // namespace agents_cpu::steering
