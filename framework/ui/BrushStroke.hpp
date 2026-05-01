#pragma once

#include <math/Vec2.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace simfw::ui {

class BrushStroke {
public:
    template <typename StampFn>
    bool paint(bool isPainting, Vec2 worldPosition, float worldRadius, StampFn&& stamp) {
        if (!isPainting) {
            m_previousPaintPosition.reset();
            return false;
        }

        const float stampSpacing = std::max(1.0f, worldRadius * 0.6f);
        bool changed = false;

        if (!m_previousPaintPosition.has_value()) {
            changed |= stamp(worldPosition);
            m_previousPaintPosition = worldPosition;
            return changed;
        }

        const Vec2 delta = worldPosition - *m_previousPaintPosition;
        const float distance = delta.length();

        if (distance >= std::numeric_limits<float>::epsilon()) {
            const Vec2 direction = delta / distance;
            for (float traveled = stampSpacing; traveled <= distance; traveled += stampSpacing) {
                changed |= stamp(*m_previousPaintPosition + direction * traveled);
            }
            if (distance < stampSpacing) {
                changed |= stamp(worldPosition);
            }
        }

        m_previousPaintPosition = worldPosition;
        return changed;
    }

private:
    std::optional<Vec2> m_previousPaintPosition;
};

} // namespace simfw::ui
