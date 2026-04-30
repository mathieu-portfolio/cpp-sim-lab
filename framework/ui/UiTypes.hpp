#pragma once

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

} // namespace simfw::ui
