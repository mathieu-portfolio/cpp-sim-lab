#pragma once

#include <math/Vec2.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace simfw::ui {

class ObstacleBrush {
public:
    template <typename AddObstacleFn>
    void paint(
        bool isPainting,
        Vec2 worldPosition,
        float worldRadius,
        AddObstacleFn&& addObstacle
    ) {
        if (!isPainting) {
            m_previousPaintPosition.reset();
            return;
        }

        const float stampSpacing = std::max(1.0f, worldRadius * 0.6f);

        if (!m_previousPaintPosition.has_value()) {
            addObstacle(worldPosition, worldRadius);
            m_previousPaintPosition = worldPosition;
            return;
        }

        const Vec2 delta = worldPosition - *m_previousPaintPosition;
        const float distance = delta.length();

        if (distance >= std::numeric_limits<float>::epsilon()) {
            const Vec2 direction = delta / distance;

            for (float traveled = stampSpacing; traveled <= distance; traveled += stampSpacing) {
                addObstacle(*m_previousPaintPosition + direction * traveled, worldRadius);
            }

            if (distance < stampSpacing) {
                addObstacle(worldPosition, worldRadius);
            }
        }

        m_previousPaintPosition = worldPosition;
    }

private:
    std::optional<Vec2> m_previousPaintPosition;
};

} // namespace simfw::ui
