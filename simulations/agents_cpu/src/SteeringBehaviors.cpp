#include "SteeringBehaviors.hpp"

#include "Simulation.hpp"

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
            "seek"
        },
        WeightedBehavior{
            BehaviorType::Separation,
            separate,
            &SimulationConfig::separationWeight,
            ForceScale::MaxForce,
            IntentMask::All,
            "separation"
        },
        WeightedBehavior{
            BehaviorType::ObstacleAvoidance,
            avoidObstacles,
            &SimulationConfig::obstacleAvoidanceWeight,
            ForceScale::MaxForce,
            IntentMask::All,
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
    Vec2 acceleration{};

    for (const WeightedBehavior& behavior : behaviors) {
        if (behavior.compute == nullptr || behavior.weight == nullptr) {
            continue;
        }

        if (!appliesToIntent(behavior, context.intent)) {
            continue;
        }

        const float weight = context.config.*(behavior.weight);

        // Behaviors still own their instrumentation. When a behavior applies to
        // the current intent, it is evaluated even at zero weight so candidate
        // scan stats remain independent from force contribution.
        const Vec2 force = behavior.compute(agentIndex, context);

        acceleration += force *
            (weight * scaleFactor(context.config, behavior.scale));
    }

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

std::optional<AgentIntent> avoidObstacleIntent(
    std::size_t agentIndex,
    BehaviorContext& context
) {
    const SimulationConfig& config = context.config;
    const Agent& agent = context.agents[agentIndex];

    for (std::size_t obstacleIndex : context.candidates.obstacles) {
        const Obstacle& obstacle = context.obstacles[obstacleIndex];
        const Vec2 away = agent.position - obstacle.position;
        const float distance = away.length();
        const float threatDistance = obstacle.radius + agent.radius +
            config.obstacleIntentRadius;

        if (distance <= threatDistance) {
            return AgentIntent::AvoidObstacle;
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
