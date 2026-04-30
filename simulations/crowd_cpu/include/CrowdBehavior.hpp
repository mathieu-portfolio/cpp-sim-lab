#pragma once

#include "Agent.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace crowd_cpu {

struct SimulationConfig;
struct SimulationStats;

struct CandidateLists {
    std::span<const std::size_t> agents;
};

struct BehaviorContext {
    const SimulationConfig& config;
    const std::vector<Agent>& agents;
    const std::vector<float>& integrationField;
    std::size_t gridWidth = 0;
    std::size_t gridHeight = 0;
    CandidateLists candidates;
    SimulationStats& stats;
};

enum class BehaviorType {
    FlowFollow,
    Separation
};

enum class ForceScale {
    Unit,
    MaxForce
};

using BehaviorFn = Vec2 (*)(std::size_t, BehaviorContext&);

struct WeightedBehavior {
    BehaviorType type = BehaviorType::FlowFollow;
    BehaviorFn compute = nullptr;
    float SimulationConfig::* weight = nullptr;
    ForceScale scale = ForceScale::Unit;
    bool enabled = true;
    const char* name = "";
};

std::vector<WeightedBehavior> makeDefaultBehaviors();
Vec2 computeAcceleration(std::size_t index, BehaviorContext& context, std::span<const WeightedBehavior> behaviors);
Vec2 limitLength(Vec2 value, float maxLen);

Vec2 followFlow(std::size_t index, BehaviorContext& context);
Vec2 separate(std::size_t index, BehaviorContext& context);

} // namespace crowd_cpu
