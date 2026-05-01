#pragma once

#include <math/Vec2.hpp>

#include <optional>

namespace traffic_flow_cpu {

class RoadBrush {
public:
    bool paint(bool isPainting, Vec2 worldPosition, float minStampSpacing) {
        if (!isPainting) {
            m_previousStampPosition.reset();
            return false;
        }

        if (!m_previousStampPosition.has_value() ||
            (worldPosition - *m_previousStampPosition).lengthSquared() >= minStampSpacing * minStampSpacing) {
            m_previousStampPosition = worldPosition;
            return true;
        }

        return false;
    }

private:
    std::optional<Vec2> m_previousStampPosition;
};

} // namespace traffic_flow_cpu
