#pragma once

#include <ui/SpatialGridDebugDraw.hpp>
#include <ui/UiHelpers.hpp>

#include <raylib.h>

namespace simfw::ui {

template <typename Config>
void handleSimulationBackendControls(
    Config& config,
    GridDebugMode& gridDebugMode
) {
    if (IsKeyPressed(KEY_G)) {
        config.execution.useSpatialGrid = !config.execution.useSpatialGrid;
    }

    if (IsKeyPressed(KEY_H)) {
        gridDebugMode = nextGridDebugMode(gridDebugMode);
    }

    if (IsKeyPressed(KEY_P)) {
        config.execution.useParallelUpdate = !config.execution.useParallelUpdate;
    }
}

template <typename Config>
void drawSimulationBackendStatus(
    TextCursor& cursor,
    const Config& config,
    GridDebugMode gridDebugMode
) {
    cursor.draw(
        config.execution.useSpatialGrid ? "Grid backend: spatial" : "Grid backend: naive",
        16,
        config.execution.useSpatialGrid ? GREEN : LIGHTGRAY
    );

    cursor.draw(
        config.execution.useParallelUpdate ? "Execution: parallel" : "Execution: single-thread",
        16,
        config.execution.useParallelUpdate ? GREEN : LIGHTGRAY
    );

    cursor.draw(
        TextFormat(
            "Grid debug: %s",
            gridDebugModeName(gridDebugMode)
        )
    );
}

} // namespace simfw::ui
