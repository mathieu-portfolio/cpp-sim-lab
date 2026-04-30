#include "SteeringBehaviors.hpp"

#include "Simulation.hpp"

#include <algorithm>
#include <simulation/ObstaclePhysics.hpp>
#include <simulation/WeightedBehaviorPipeline.hpp>
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

IntentMask maskForIntent(AgentIntent intent) {
    switch (intent) {
    case AgentIntent::SeekTarget:
        return IntentMask::SeekTarget;
    case AgentIntent::AvoidObstacle:
        return IntentMask::AvoidObstacle;
    case AgentIntent::Idle:
        return IntentMask::Idle;
    }

    return IntentMask::None;
}

BehaviorFn behaviorFunction(BehaviorType type) {
    switch (type) {
    case BehaviorType::Seek:
        return seek;
    case BehaviorType::Separation:
        return separate;
    case BehaviorType::ObstacleAvoidance:
        return avoidObstacles;
    }

    return nullptr;
}

const char* behaviorName(BehaviorType type) {
    switch (type) {
    case BehaviorType::Seek:
        return "seek";
    case BehaviorType::Separation:
        return "separation";
    case BehaviorType::ObstacleAvoidance:
        return "obstacle avoidance";
    }

    return "unknown";
}
} // namespace

std::span<const WeightedBehavior> defaultBehaviors() {
    static constexpr std::array<WeightedBehavior, 3> Behaviors{{
        WeightedBehavior{
            BehaviorType::Seek,
            seek,
            &SimulationConfig::seekWeight,
            ForceScale::Unit,
            IntentMask::Moving,
            true,
            "seek"
        },
        WeightedBehavior{
            BehaviorType::Separation,
            separate,
            &SimulationConfig::separationWeight,
            ForceScale::MaxForce,
            IntentMask::All,
            true,
            "separation"
        },
        WeightedBehavior{
            BehaviorType::ObstacleAvoidance,
            avoidObstacles,
            &SimulationConfig::obstacleAvoidanceWeight,
            ForceScale::MaxForce,
            IntentMask::All,
            true,
            "obstacle avoidance"
        }
    }};

    return Behaviors;
}

std::span<const IntentRule> defaultIntentRules() {
    static constexpr std::array<IntentRule, 3> Rules{{
        IntentRule{avoidObstacleIntent, "avoid obstacle"},
        IntentRule{idleAtTargetIntent, "idle at target"},
        IntentRule{seekTargetIntent, "seek target"}
    }};

    return Rules;
}

std::vector<WeightedBehavior> makeDefaultBehaviors() {
    const auto defaults = defaultBehaviors();
    return std::vector<WeightedBehavior>{defaults.begin(), defaults.end()};
}

std::vector<IntentRule> makeDefaultIntentRules() {
    const auto defaults = defaultIntentRules();
    return std::vector<IntentRule>{defaults.begin(), defaults.end()};
}

WeightedBehavior makeBehavior(
    BehaviorType type,
    float SimulationConfig::* weight,
    ForceScale scale,
    IntentMask intents
) {
    return WeightedBehavior{
        type,
        behaviorFunction(type),
        weight,
        scale,
        intents,
        true,
        behaviorName(type)
    };
}

bool appliesToIntent(const WeightedBehavior& behavior, AgentIntent intent) {
    return any(behavior.intents & maskForIntent(intent));
}

AgentIntent selectIntent(
    std::size_t agentIndex,
    BehaviorContext& context,
    std::span<const IntentRule> intentRules
) {
    if (!context.config.useIntent) {
        return AgentIntent::SeekTarget;
    }

    for (const IntentRule& rule : intentRules) {
        if (rule.evaluate == nullptr) {
            continue;
        }

        if (std::optional<AgentIntent> intent = rule.evaluate(agentIndex, context)) {
            return *intent;
        }
    }

    return AgentIntent::SeekTarget;
}

void recordIntentStats(
    AgentIntent intent,
    AgentIntent previousIntent,
    SimulationStats& stats
) {
    switch (intent) {
    case AgentIntent::SeekTarget:
        ++stats.seekingTargetCount;
        break;
    case AgentIntent::AvoidObstacle:
        ++stats.avoidingObstacleCount;
        break;
    case AgentIntent::Idle:
        ++stats.idleCount;
        break;
    }

    if (intent != previousIntent) {
        ++stats.intentChanges;
    }
}

Vec2 limitLength(Vec2 value, float maxLength) {
    const float length = value.length();

    if (length <= maxLength || length <= Epsilon) {
        return value;
    }

    return value * (maxLength / length);
}

Vec2 computeAcceleration(
    std::size_t agentIndex,
    BehaviorContext& context,
    std::span<const WeightedBehavior> behaviors
) {
    const Vec2 acceleration = simfw::simulation::computeWeightedBehaviors<
        WeightedBehavior,
        BehaviorContext,
        Vec2
    >(
        agentIndex,
        behaviors,
        context,
        [](const WeightedBehavior& behavior, const BehaviorContext& ctx) {
            if (behavior.weight == nullptr) {
                return 0.0f;
            }

            return ctx.config.*(behavior.weight);
        },
        [](const WeightedBehavior& behavior, const BehaviorContext& ctx) {
            return appliesToIntent(behavior, ctx.intent);
        },
        [](const WeightedBehavior& behavior, const BehaviorContext& ctx) {
            return scaleFactor(ctx.config, behavior.scale);
        }
    );

    return limitLength(acceleration, context.config.maxForce);
}

Vec2 seek(std::size_t agentIndex, BehaviorContext& context) {
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

    const float step = std::max(4.0f, config.obstacleAvoidanceRadius * 0.25f);
    for (float y = -config.obstacleAvoidanceRadius; y <= config.obstacleAvoidanceRadius; y += step) {
        for (float x = -config.obstacleAvoidanceRadius; x <= config.obstacleAvoidanceRadius; x += step) {
            const Vec2 delta{x, y};
            const float distance = delta.length();
            if (distance <= Epsilon || distance >= config.obstacleAvoidanceRadius) continue;
            ++context.stats.obstacleChecks;
            if (!simfw::simulation::isBlockedWorld(context.obstacleMask, agent.position + delta)) continue;
            const float strength = 1.0f - (distance / config.obstacleAvoidanceRadius);
            force += (delta * -1.0f).normalized() * strength;
            ++obstacleCount;
        }
    }

    if (obstacleCount > 0) {
        force *= 1.0f / static_cast<float>(obstacleCount);
    }

    return force;
}

std::optional<AgentIntent> avoidObstacleIntent(
    std::size_t agentIndex,
    BehaviorContext& context
) {
    const SimulationConfig& config = context.config;
    const Agent& agent = context.agents[agentIndex];

    const float step = std::max(4.0f, config.obstacleIntentRadius * 0.25f);
    for (float y = -config.obstacleIntentRadius; y <= config.obstacleIntentRadius; y += step) {
        for (float x = -config.obstacleIntentRadius; x <= config.obstacleIntentRadius; x += step) {
            const Vec2 delta{x, y};
            if (delta.length() > config.obstacleIntentRadius) continue;
            if (simfw::simulation::isBlockedWorld(context.obstacleMask, agent.position + delta)) return AgentIntent::AvoidObstacle;
        }
    }

    return std::nullopt;
}

std::optional<AgentIntent> idleAtTargetIntent(
    std::size_t agentIndex,
    BehaviorContext& context
) {
    const SimulationConfig& config = context.config;
    const Agent& agent = context.agents[agentIndex];

    if ((agent.target - agent.position).length() <= config.targetRadius) {
        return AgentIntent::Idle;
    }

    return std::nullopt;
}

std::optional<AgentIntent> seekTargetIntent(
    std::size_t,
    BehaviorContext&
) {
    return AgentIntent::SeekTarget;
}

void resolveObstacleOverlap(
    Agent& agent,
    const Vec2& previousPosition,
    const simfw::simulation::ObstacleMask& obstacleMask,
    SimulationStats&
) {
    const Vec2 proposedPosition = agent.position;
    const simfw::simulation::ObstacleCollisionResult collision = simfw::simulation::resolveObstacleCollisionWithNormal(
        obstacleMask,
        previousPosition,
        proposedPosition,
        agent.radius
    );
    const Vec2 resolvedPosition = collision.position;
    agent.position = resolvedPosition;

    if (!collision.collided || collision.normal.lengthSquared() <= 1e-6f) {
        return;
    }

    const float normalSpeed = agent.velocity.dot(collision.normal);
    if (normalSpeed < 0.0f) {
        constexpr float restitution = 0.35f;
        agent.velocity -= collision.normal * ((1.0f + restitution) * normalSpeed);
    }
}

} // namespace agents_cpu::steering
