#pragma once

#include "Simulation.hpp"
#include <ui/BrushStroke.hpp>

namespace traffic_flow_cpu {

class RoadBrush {
public:
    bool paint(Simulation& sim, bool isPainting, Vec2 worldPosition, float worldRadius, bool hasRoad) {
        return m_stroke.paint(isPainting, worldPosition, worldRadius, [&](Vec2 point) {
            return sim.paintRoadAtWorld(point, worldRadius, hasRoad);
        });
    }

private:
    simfw::ui::BrushStroke m_stroke;
};

} // namespace traffic_flow_cpu
