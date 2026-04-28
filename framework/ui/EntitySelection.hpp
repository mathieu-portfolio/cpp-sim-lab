#pragma once

#include <math/Vec2.hpp>

#include <cstddef>
#include <optional>

#include <raylib.h>

namespace simfw::ui {

inline Vec2 screenToWorld(Vec2 screenPosition, const Camera2D& camera) {
    const Vector2 world = GetScreenToWorld2D(
        Vector2{screenPosition.x, screenPosition.y},
        camera
    );

    return Vec2{world.x, world.y};
}

inline Vec2 screenToWorld(Vector2 screenPosition, const Camera2D& camera) {
    return screenToWorld(Vec2{screenPosition.x, screenPosition.y}, camera);
}

template <typename Entities, typename PositionFn>
std::optional<std::size_t> findClosestEntity(
    const Entities& entities,
    Vec2 worldPosition,
    float maxDistance,
    PositionFn positionFor
) {
    std::optional<std::size_t> bestIndex;
    float bestDistanceSquared = maxDistance * maxDistance;

    for (std::size_t i = 0; i < entities.size(); ++i) {
        const Vec2 offset = positionFor(entities[i]) - worldPosition;
        const float distanceSquared = Vec2::dot(offset, offset);

        if (distanceSquared <= bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            bestIndex = i;
        }
    }

    return bestIndex;
}

inline void drawSelectionRing(
    Vec2 position,
    float radius,
    Color color = YELLOW
) {
    DrawCircleLines(
        static_cast<int>(position.x),
        static_cast<int>(position.y),
        radius,
        color
    );
}

} // namespace simfw::ui
