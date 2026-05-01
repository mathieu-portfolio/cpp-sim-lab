#pragma once

#include <ui/BrushStroke.hpp>

#include <algorithm>
#include <optional>

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
        if (!isPainting) {
            m_previousStampPosition.reset();
            return m_stroke.paint(false, worldPosition, worldRadius, [](Vec2) { return false; });
        }

        const float safePixelsPerUnit = std::max(1.0f, roadPixelsPerSimUnit);
        return m_stroke.paint(isPainting, worldPosition, worldRadius, [&](Vec2 point) {
            bool changed = false;
            if (m_previousStampPosition.has_value()) {
                const float deltaPixels = (point - *m_previousStampPosition).length();
                const float deltaRoadUnits = deltaPixels / safePixelsPerUnit;
                const float extendedLength = std::clamp(roadLength + deltaRoadUnits, minRoadLength, maxRoadLength);
                if (extendedLength > roadLength) {
                    roadLength = extendedLength;
                    changed = true;
                }
            } else {
                const float projectedRoadLength = (point.x - roadOriginX) / safePixelsPerUnit;
                const float clampedRoadLength = std::clamp(projectedRoadLength, minRoadLength, maxRoadLength);
                if (clampedRoadLength > roadLength) {
                    roadLength = clampedRoadLength;
                    changed = true;
                }
            }

            m_previousStampPosition = point;
            return changed;
        });
    }

private:
    simfw::ui::BrushStroke m_stroke;
    std::optional<Vec2> m_previousStampPosition;
};

} // namespace traffic_flow_cpu
