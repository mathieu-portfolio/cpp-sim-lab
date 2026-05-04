#pragma once

#include <ui/SpatialGridDebugDraw.hpp>
#include <ui/UiHelpers.hpp>

#include <algorithm>
#include <initializer_list>

#include <raylib.h>

namespace simfw::ui {

inline TextCursor makeRightSideControlCursor(
    int panelWidth = 300,
    int top = 10,
    int lineHeight = 20
) {
    return TextCursor{
        std::max(10, GetScreenWidth() - panelWidth),
        top,
        lineHeight
    };
}

inline void drawControlHints(
    TextCursor& cursor,
    std::initializer_list<const char*> controls,
    Color headingColor = RAYWHITE,
    Color controlColor = LIGHTGRAY
) {
    cursor.draw("Controls", 18, headingColor);
    cursor.gap(4);

    for (const char* control : controls) {
        cursor.draw(control, 16, controlColor);
    }
}

} // namespace simfw::ui
