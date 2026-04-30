#pragma once

#include "UiTypes.hpp"

#include <cstddef>

namespace simfw::ui {

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
