#pragma once

#include "UiTypes.hpp"

#include <algorithm>

#include <math/Vec2.hpp>

namespace simfw::ui {

inline Vector2 toRaylib(Vec2 value) {
    return Vector2{value.x, value.y};
}

inline float clampMin(float value, float minValue) {
    return std::max(value, minValue);
}

inline UiMode nextUiMode(UiMode mode) {
    return static_cast<UiMode>((static_cast<int>(mode) + 1) % 3);
}

inline GridDebugMode nextGridDebugMode(GridDebugMode mode) {
    return static_cast<GridDebugMode>((static_cast<int>(mode) + 1) % 3);
}

inline const char* gridDebugModeName(GridDebugMode mode) {
    switch (mode) {
        case GridDebugMode::None:
            return "none";
        case GridDebugMode::OccupiedCells:
            return "occupied cells";
        case GridDebugMode::HotCells:
            return "hot cells";
    }

    return "unknown";
}

inline void adjustTunable(
    TunableParameter& parameter,
    float direction,
    float dt,
    bool fast
) {
    if (parameter.value == nullptr || direction == 0.0f) {
        return;
    }

    const float step = (fast ? parameter.fastStep : parameter.normalStep) * dt;
    *parameter.value = clampMin(
        *parameter.value + direction * step,
        parameter.minValue
    );
}

} // namespace simfw::ui
