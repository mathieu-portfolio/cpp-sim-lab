#pragma once

#include "Agent.hpp"
#include "Obstacle.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace agents_cpu {

struct SimulationConfig;
struct SimulationStats;

namespace steering {

struct CandidateLists {
    std::span<const std::size_t> agents;
    std::span<const std::size_t> obstacles;
};

struct BehaviorContext {
    const SimulationConfig& config;
    const std::vector<Agent>& agents;
    const std::vector<Obstacle>& obstacles;
    CandidateLists candidates;
    AgentIntent intent = AgentIntent::SeekTarget;
    SimulationStats& stats;
};

using BehaviorFn = Vec2 (*)(std::size_t agentIndex, BehaviorContext& context);
using IntentRuleFn = std::optional<AgentIntent> (*)(
    std::size_t agentIndex,
    BehaviorContext& context
);

enum class BehaviorType {
    Seek,
    Separation,
    ObstacleAvoidance
};

enum class ForceScale {
    Unit,
    MaxForce
};

enum class IntentMask : std::uint8_t {
    None = 0,
    SeekTarget = 1u << 0u,
    AvoidObstacle = 1u << 1u,
    Idle = 1u << 2u,
    Moving = SeekTarget | AvoidObstacle,
    All = SeekTarget | AvoidObstacle | Idle
};

constexpr IntentMask operator|(IntentMask lhs, IntentMask rhs) {
    return static_cast<IntentMask>(
        static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs)
    );
}

constexpr IntentMask operator&(IntentMask lhs, IntentMask rhs) {
    return static_cast<IntentMask>(
        static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs)
    );
}

constexpr bool any(IntentMask mask) {
    return static_cast<std::uint8_t>(mask) != 0u;
}

struct WeightedBehavior {
    BehaviorType type = BehaviorType::Seek;
    BehaviorFn compute = nullptr;
    float SimulationConfig::* weight = nullptr;
    ForceScale scale = ForceScale::Unit;
    IntentMask intents = IntentMask::All;
    bool enabled = true;
    const char* name = "";
};

struct IntentRule {
    IntentRuleFn evaluate = nullptr;
    const char* name = "";
};

std::span<const WeightedBehavior> defaultBehaviors();
std::span<const IntentRule> defaultIntentRules();

std::vector<WeightedBehavior> makeDefaultBehaviors();
std::vector<IntentRule> makeDefaultIntentRules();

WeightedBehavior makeBehavior(
    BehaviorType type,
    float SimulationConfig::* weight,
    ForceScale scale,
    IntentMask intents = IntentMask::All
);

bool appliesToIntent(const WeightedBehavior& behavior, AgentIntent intent);

AgentIntent selectIntent(
    std::size_t agentIndex,
    BehaviorContext& context,
    std::span<const IntentRule> intentRules
);

void recordIntentStats(
    AgentIntent intent,
    AgentIntent previousIntent,
    SimulationStats& stats
);

Vec2 limitLength(Vec2 value, float maxLength);

Vec2 computeAcceleration(
    std::size_t agentIndex,
    BehaviorContext& context,
    std::span<const WeightedBehavior> behaviors
);

Vec2 seek(std::size_t agentIndex, BehaviorContext& context);
Vec2 separate(std::size_t agentIndex, BehaviorContext& context);
Vec2 avoidObstacles(std::size_t agentIndex, BehaviorContext& context);

std::optional<AgentIntent> avoidObstacleIntent(
    std::size_t agentIndex,
    BehaviorContext& context
);

std::optional<AgentIntent> idleAtTargetIntent(
    std::size_t agentIndex,
    BehaviorContext& context
);

std::optional<AgentIntent> seekTargetIntent(
    std::size_t agentIndex,
    BehaviorContext& context
);

void resolveObstacleOverlap(
    Agent& agent,
    const std::vector<Obstacle>& obstacles,
    std::span<const std::size_t> obstacleCandidates,
    SimulationStats& stats
);

} // namespace steering
} // namespace agents_cpu
