#pragma once

#include <math/Vec2.hpp>
#include <simulation/ObstacleMask.hpp>

#include <algorithm>
#include <cmath>

namespace simfw::simulation {
struct ObstacleCollisionResult {
    Vec2 position;
    Vec2 normal;
    bool collided = false;
};

inline bool isBlockedWorld(const ObstacleMask& mask, Vec2 worldPos) {
    if (mask.empty()) {
        return false;
    }
    const int x = std::clamp(static_cast<int>(worldPos.x), 0, mask.width() - 1);
    const int y = std::clamp(static_cast<int>(worldPos.y), 0, mask.height() - 1);
    return mask.isBlocked(x, y);
}

inline ObstacleCollisionResult resolveObstacleCollisionWithNormal(
    const ObstacleMask& mask,
    Vec2 previousPos,
    Vec2 proposedPos,
    float radius
) {
    if (mask.empty()) {
        return ObstacleCollisionResult{proposedPos, Vec2{0.0f, 0.0f}, false};
    }

    Vec2 resolved = proposedPos;
    Vec2 accumulatedNormal{0.0f, 0.0f};
    bool anyCollision = false;
    const float padding = 0.25f;
    const float expanded = radius + padding;
    const float expandedSq = expanded * expanded;

    for (int iteration = 0; iteration < 3; ++iteration) {
        bool collided = false;
        const int minX = std::max(0, static_cast<int>(std::floor(resolved.x - expanded)));
        const int maxX = std::min(mask.width() - 1, static_cast<int>(std::ceil(resolved.x + expanded)));
        const int minY = std::max(0, static_cast<int>(std::floor(resolved.y - expanded)));
        const int maxY = std::min(mask.height() - 1, static_cast<int>(std::ceil(resolved.y + expanded)));

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                if (!mask.isBlocked(x, y)) {
                    continue;
                }
                const float minCellX = static_cast<float>(x);
                const float maxCellX = minCellX + 1.0f;
                const float minCellY = static_cast<float>(y);
                const float maxCellY = minCellY + 1.0f;
                const float closestX = std::clamp(resolved.x, minCellX, maxCellX);
                const float closestY = std::clamp(resolved.y, minCellY, maxCellY);
                Vec2 delta = resolved - Vec2{closestX, closestY};
                float distSq = delta.lengthSquared();
                if (distSq >= expandedSq) {
                    continue;
                }

                collided = true;
                anyCollision = true;
                if (distSq <= 1e-8f) {
                    delta = (resolved - previousPos).normalized();
                    if (delta.lengthSquared() <= 1e-8f) {
                        delta = Vec2{1.0f, 0.0f};
                    }
                    distSq = 1e-8f;
                }

                const float dist = std::sqrt(distSq);
                const float penetration = expanded - dist;
                const Vec2 normal = delta / dist;
                resolved += normal * penetration;
                accumulatedNormal += normal;
            }
        }

        if (!collided) {
            const Vec2 normal = accumulatedNormal.lengthSquared() > 1e-8f ? accumulatedNormal.normalized() : Vec2{0.0f, 0.0f};
            return ObstacleCollisionResult{resolved, normal, anyCollision};
        }
    }

    Vec2 fallback = previousPos;
    if (isBlockedWorld(mask, fallback)) {
        constexpr int maxSearchRadius = 64;
        const int originX = std::clamp(static_cast<int>(std::round(proposedPos.x)), 0, mask.width() - 1);
        const int originY = std::clamp(static_cast<int>(std::round(proposedPos.y)), 0, mask.height() - 1);
        bool found = false;
        for (int r = 1; r <= maxSearchRadius && !found; ++r) {
            for (int y = std::max(0, originY - r); y <= std::min(mask.height() - 1, originY + r) && !found; ++y) {
                for (int x = std::max(0, originX - r); x <= std::min(mask.width() - 1, originX + r); ++x) {
                    if (mask.isBlocked(x, y)) {
                        continue;
                    }
                    fallback = Vec2{static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f};
                    found = true;
                    break;
                }
            }
        }
    }
    const Vec2 fallbackNormal = (fallback - proposedPos).lengthSquared() > 1e-8f ? (fallback - proposedPos).normalized() : Vec2{0.0f, 0.0f};
    return ObstacleCollisionResult{fallback, fallbackNormal, true};
}

inline Vec2 resolveObstacleCollision(
    const ObstacleMask& mask,
    Vec2 previousPos,
    Vec2 proposedPos,
    float radius
) {
    return resolveObstacleCollisionWithNormal(mask, previousPos, proposedPos, radius).position;
}

} // namespace simfw::simulation
