#include "CrowdBehavior.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <simulation/WeightedBehaviorPipeline.hpp>

namespace crowd_cpu {
namespace {
constexpr float Epsilon = 0.0001f;

float scaleFactor(const SimulationConfig& config, ForceScale scale) {
    return scale == ForceScale::MaxForce ? config.maxForce : 1.0f;
}

std::size_t flowCellIndex(const SimulationConfig& config, Vec2 p, std::size_t w, std::size_t h) {
    const float invCell = 1.0f / config.gridCellSize;
    const auto cx = static_cast<std::size_t>(std::clamp(static_cast<int>(p.x * invCell), 0, static_cast<int>(w - 1)));
    const auto cy = static_cast<std::size_t>(std::clamp(static_cast<int>(p.y * invCell), 0, static_cast<int>(h - 1)));
    return cy * w + cx;
}
}

std::vector<WeightedBehavior> makeDefaultBehaviors() {
    return {
        {BehaviorType::FlowFollow, followFlow, &SimulationConfig::flowWeight, ForceScale::Unit, true, "flow"},
        {BehaviorType::Separation, separate, &SimulationConfig::separationWeight, ForceScale::MaxForce, true, "separation"},
        {BehaviorType::ObstacleAvoidance, avoidObstacles, &SimulationConfig::obstacleAvoidanceWeight, ForceScale::MaxForce, true, "obstacle"}
    };
}

Vec2 limitLength(Vec2 v, float maxLen) { float l=v.length(); return (l<=maxLen||l<=Epsilon)?v:v*(maxLen/l); }

Vec2 computeAcceleration(std::size_t i, BehaviorContext& c, std::span<const WeightedBehavior> b) {
    const Vec2 force = simfw::simulation::computeWeightedBehaviors<WeightedBehavior, BehaviorContext, Vec2>(
        i, b, c,
        [](const WeightedBehavior& wb, const BehaviorContext& ctx){ return wb.weight ? ctx.config.*(wb.weight) : 0.0f; },
        simfw::simulation::AlwaysApplyBehavior{},
        [](const WeightedBehavior& wb, const BehaviorContext& ctx){ return scaleFactor(ctx.config, wb.scale); }
    );
    return limitLength(force, c.config.maxForce);
}

Vec2 followFlow(std::size_t index, BehaviorContext& context) {
    const Agent& a = context.agents[index];
    if (context.flowVectors.empty()) return Vec2{};
    const std::size_t total = context.flowVectors.size();
    const std::size_t w = static_cast<std::size_t>(context.config.width / context.config.gridCellSize) + 1;
    const std::size_t h = std::max<std::size_t>(1, total / std::max<std::size_t>(1, w));
    Vec2 dir = context.flowVectors[std::min(total - 1, flowCellIndex(context.config, a.position, w, h))];
    if (dir.length() <= Epsilon) return Vec2{};
    Vec2 desired = dir.normalized() * context.config.maxSpeed;
    return limitLength(desired - a.velocity, context.config.maxForce);
}

Vec2 separate(std::size_t index, BehaviorContext& c) {
    Vec2 out{}; int count=0;
    for (std::size_t n : c.candidates.agents) {
        ++c.stats.neighborChecks;
        Vec2 away = c.agents[index].position - c.agents[n].position;
        float d = away.length();
        if (d <= Epsilon || d >= c.config.separationRadius) continue;
        out += away.normalized() * (1.0f - d / c.config.separationRadius);
        ++count;
    }
    if (count > 0) out *= 1.0f / static_cast<float>(count);
    return out;
}

Vec2 avoidObstacles(std::size_t index, BehaviorContext& c) {
    Vec2 out{};
    for (std::size_t oi : c.candidates.obstacles) {
        ++c.stats.obstacleChecks;
        Vec2 away = c.agents[index].position - c.obstacles[oi].position;
        float d = away.length();
        float r = c.obstacles[oi].radius + c.config.obstacleAvoidanceRadius;
        if (d <= Epsilon || d >= r) continue;
        out += away.normalized() * (1.0f - d / r);
    }
    return out;
}

} // namespace crowd_cpu
