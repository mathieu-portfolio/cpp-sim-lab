#include "CrowdBehavior.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <simulation/WeightedBehaviorPipeline.hpp>

namespace crowd_cpu {
namespace {
constexpr float Epsilon = 0.0001f;

float scaleFactor(const SimulationConfig& config, ForceScale scale) {
    return scale == ForceScale::MaxForce ? config.maxForce : 1.0f;
}

float sampleIntegrationBilinear(
    const std::vector<float>& integration,
    std::size_t gridWidth,
    std::size_t gridHeight,
    float cellSize,
    Vec2 worldPos
) {
    if (integration.empty() || gridWidth == 0 || gridHeight == 0) {
        return std::numeric_limits<float>::infinity();
    }

    const float gx = std::clamp(worldPos.x / cellSize, 0.0f, static_cast<float>(gridWidth - 1));
    const float gy = std::clamp(worldPos.y / cellSize, 0.0f, static_cast<float>(gridHeight - 1));
    const std::size_t x0 = static_cast<std::size_t>(gx);
    const std::size_t y0 = static_cast<std::size_t>(gy);
    const std::size_t x1 = std::min(x0 + 1, gridWidth - 1);
    const std::size_t y1 = std::min(y0 + 1, gridHeight - 1);
    const float tx = gx - static_cast<float>(x0);
    const float ty = gy - static_cast<float>(y0);

    const float i00 = integration[y0 * gridWidth + x0];
    const float i10 = integration[y0 * gridWidth + x1];
    const float i01 = integration[y1 * gridWidth + x0];
    const float i11 = integration[y1 * gridWidth + x1];
    if (!std::isfinite(i00) || !std::isfinite(i10) || !std::isfinite(i01) || !std::isfinite(i11)) {
        return std::numeric_limits<float>::infinity();
    }

    const float top = i00 + (i10 - i00) * tx;
    const float bottom = i01 + (i11 - i01) * tx;
    return top + (bottom - top) * ty;
}

Vec2 sampleFlowFromIntegration(
    const std::vector<float>& integration,
    std::size_t gridWidth,
    std::size_t gridHeight,
    float cellSize,
    Vec2 worldPos
) {
    const float h = cellSize;
    const float fxc = sampleIntegrationBilinear(integration, gridWidth, gridHeight, cellSize, worldPos);
    const float fxm = sampleIntegrationBilinear(integration, gridWidth, gridHeight, cellSize, Vec2{worldPos.x - h, worldPos.y});
    const float fxp = sampleIntegrationBilinear(integration, gridWidth, gridHeight, cellSize, Vec2{worldPos.x + h, worldPos.y});
    const float fym = sampleIntegrationBilinear(integration, gridWidth, gridHeight, cellSize, Vec2{worldPos.x, worldPos.y - h});
    const float fyp = sampleIntegrationBilinear(integration, gridWidth, gridHeight, cellSize, Vec2{worldPos.x, worldPos.y + h});

    if (!std::isfinite(fxc)) {
        return Vec2{};
    }

    float ddx = 0.0f;
    if (std::isfinite(fxp) && std::isfinite(fxm)) {
        ddx = (fxp - fxm) / (2.0f * h);
    } else if (std::isfinite(fxp)) {
        ddx = (fxp - fxc) / h;
    } else if (std::isfinite(fxm)) {
        ddx = (fxc - fxm) / h;
    }

    float ddy = 0.0f;
    if (std::isfinite(fyp) && std::isfinite(fym)) {
        ddy = (fyp - fym) / (2.0f * h);
    } else if (std::isfinite(fyp)) {
        ddy = (fyp - fxc) / h;
    } else if (std::isfinite(fym)) {
        ddy = (fxc - fym) / h;
    }

    return Vec2{-ddx, -ddy};
}
}

std::vector<WeightedBehavior> makeDefaultBehaviors() {
    return {
        {BehaviorType::FlowFollow, followFlow, &SimulationConfig::flowWeight, ForceScale::Unit, true, "flow"},
        {BehaviorType::Separation, separate, &SimulationConfig::separationWeight, ForceScale::MaxForce, true, "separation"},
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
    Vec2 dir = sampleFlowFromIntegration(
        context.integrationField,
        context.gridWidth,
        context.gridHeight,
        context.config.gridCellSize,
        a.position
    );
    if (dir.lengthSquared() <= Epsilon * Epsilon) return Vec2{};
    Vec2 desired = dir.normalized() * context.config.maxSpeed;
    return limitLength(desired - a.velocity, context.config.maxForce);
}

Vec2 separate(std::size_t index, BehaviorContext& c) {
    Vec2 out{};

    const Vec2 selfPos = c.agents[index].position;
    const float radius = c.config.separationRadius;
    const float radiusSq = radius * radius;

    for (std::size_t n : c.candidates.agents) {
        if (n == index) continue;

        ++c.stats.neighborChecks;

        Vec2 away = selfPos - c.agents[n].position;
        float dSq = away.lengthSquared();

        if (dSq <= Epsilon * Epsilon || dSq >= radiusSq) continue;

        float d = std::sqrt(dSq);
        float falloff = 1.0f - d / radius;

        out += away * (falloff / dSq);
    }

    return out;
}


} // namespace crowd_cpu
