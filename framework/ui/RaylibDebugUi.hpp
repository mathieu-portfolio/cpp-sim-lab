#pragma once

#include <algorithm>
#include <cstddef>

#include <math/Vec2.hpp>
#include <raylib.h>

namespace simfw::ui {

enum class UiMode {
    None,
    Compact,
    Full
};

enum class GridDebugMode {
    None,
    OccupiedCells,
    HotCells
};

struct TunableParameter {
    const char* name = "";
    float* value = nullptr;
    float minValue = 0.0f;
    float normalStep = 1.0f;
    float fastStep = 5.0f;
};

struct TextCursor {
    int x = 10;
    int y = 10;
    int lineHeight = 20;

    void draw(
        const char* text,
        int fontSize = 16,
        Color color = LIGHTGRAY
    ) {
        DrawText(text, x, y, fontSize, color);
        y += lineHeight;
    }

    void gap(int pixels) {
        y += pixels;
    }
};

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

template <typename Grid>
void drawSpatialGridDebug(
    const Grid& grid,
    GridDebugMode mode,
    std::size_t hotCellThreshold = 4,
    Color cellColor = DARKGREEN,
    Color textColor = GREEN
) {
    if (mode == GridDebugMode::None) {
        return;
    }

    const float cellSize = grid.getCellSize();

    for (const auto& [coord, indices] : grid.getCells()) {
        if (mode == GridDebugMode::HotCells && indices.size() < hotCellThreshold) {
            continue;
        }

        const int x = static_cast<int>(coord.x * cellSize);
        const int y = static_cast<int>(coord.y * cellSize);
        const int size = static_cast<int>(cellSize);

        DrawRectangleLines(x, y, size, size, cellColor);

        if (indices.size() >= 2) {
            DrawText(
                TextFormat("%d", static_cast<int>(indices.size())),
                x + 4,
                y + 4,
                12,
                textColor
            );
        }
    }
}

} // namespace simfw::ui
