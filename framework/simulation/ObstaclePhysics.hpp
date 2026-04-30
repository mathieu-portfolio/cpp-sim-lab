#pragma once

#include <math/Vec2.hpp>
#include <simulation/ObstacleMask.hpp>

#include <algorithm>
#include <cmath>

namespace simfw::simulation {

inline bool isBlockedWorld(const ObstacleMask& mask, Vec2 worldPos) {
    if (mask.empty()) {
        return false;
    }
    const int x = std::clamp(static_cast<int>(worldPos.x), 0, mask.width() - 1);
    const int y = std::clamp(static_cast<int>(worldPos.y), 0, mask.height() - 1);
    return mask.isBlocked(x, y);
}

inline Vec2 resolveObstacleCollision(
    const ObstacleMask& mask,
    Vec2 previousPos,
    Vec2 proposedPos,
    float radius
) {
    if (mask.empty()) {
        return proposedPos;
    }

    Vec2 resolved = proposedPos;
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
                const Vec2 cellCenter{static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f};
                Vec2 delta = resolved - cellCenter;
                float distSq = delta.lengthSquared();
                if (distSq >= expandedSq) {
                    continue;
                }

                collided = true;
                if (distSq <= 1e-8f) {
                    delta = (resolved - previousPos).normalized();
                    if (delta.lengthSquared() <= 1e-8f) {
                        delta = Vec2{1.0f, 0.0f};
                    }
                    distSq = 1e-8f;
                }

                const float dist = std::sqrt(distSq);
                const float penetration = expanded - dist;
                resolved += delta / dist * penetration;
            }
        }

        if (!collided) {
            return resolved;
        }
    }

    return previousPos;
}

} // namespace simfw::simulation
