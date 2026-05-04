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

inline float clampRange(float value, float minValue, float maxValue) {
    return std::clamp(value, minValue, maxValue);
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

    const float appliedStep = parameter.step * (fast ? parameter.fastMultiplier : 1.0f) * dt;
    *parameter.value = clampRange(
        *parameter.value + direction * appliedStep,
        parameter.minValue,
        parameter.maxValue
    );
}

} // namespace simfw::ui
