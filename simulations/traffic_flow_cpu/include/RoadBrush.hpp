#pragma once

#include <ui/BrushStroke.hpp>

#include <algorithm>

namespace traffic_flow_cpu {

class RoadBrush {
public:
    bool paint(
        bool isPainting,
        Vec2 worldPosition,
        float worldRadius,
        float roadOriginX,
        float roadPixelsPerSimUnit,
        float minRoadLength,
        float maxRoadLength,
        float& roadLength
    ) {
        return m_stroke.paint(isPainting, worldPosition, worldRadius, [&](Vec2 point) {
            const float projectedRoadLength = (point.x - roadOriginX) / roadPixelsPerSimUnit;
            const float clampedRoadLength = std::clamp(projectedRoadLength, minRoadLength, maxRoadLength);
            if (clampedRoadLength > roadLength) {
                roadLength = clampedRoadLength;
                return true;
            }
            return false;
        });
    }

private:
    simfw::ui::BrushStroke m_stroke;
};

} // namespace traffic_flow_cpu
