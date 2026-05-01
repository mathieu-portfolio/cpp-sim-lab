#pragma once

#include <ui/BrushStroke.hpp>
#include <math/Vec2.hpp>
#include <simulation/ObstacleMask.hpp>

namespace simfw::ui {

class ObstacleBrush {
public:
    bool paint(
        simfw::simulation::ObstacleMask& mask,
        bool isPainting,
        Vec2 worldPosition,
        float worldRadius,
        simfw::simulation::ObstaclePaintMode mode
    ) {
        return m_stroke.paint(isPainting, worldPosition, worldRadius, [&](Vec2 point) {
            return mask.paintCircle(point, worldRadius, mode);
        });
    }

private:
    BrushStroke m_stroke;
};

} // namespace simfw::ui
