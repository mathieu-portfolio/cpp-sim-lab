#include "Simulation.hpp"

#include <algorithm>
#include <array>
#include <raylib.h>

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;

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
    const char* name;
    float* value;
    float minValue;
    float normalStep;
    float fastStep;
};

Vector2 toRaylib(Vec2 v) {
    return Vector2{v.x, v.y};
}

float clampMin(float value, float minValue) {
    return std::max(value, minValue);
}

UiMode nextUiMode(UiMode mode) {
    return static_cast<UiMode>((static_cast<int>(mode) + 1) % 3);
}

GridDebugMode nextGridDebugMode(GridDebugMode mode) {
    return static_cast<GridDebugMode>((static_cast<int>(mode) + 1) % 3);
}

const char* gridDebugModeName(GridDebugMode mode) {
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

void drawBoid(const Boid& b) {
    DrawCircleV(toRaylib(b.position), 3.0f, WHITE);

    Vec2 dir = b.velocity.length() > 0.001f
        ? b.velocity.normalized()
        : Vec2{1.0f, 0.0f};

    Vec2 end = b.position + dir * 10.0f;

    DrawLineV(toRaylib(b.position), toRaylib(end), GRAY);
}

void drawDebugRadii(const Boid& b, const SimulationConfig& config) {
    DrawCircleLines(
        static_cast<int>(b.position.x),
        static_cast<int>(b.position.y),
        config.perceptionRadius,
        DARKGRAY
    );

    DrawCircleLines(
        static_cast<int>(b.position.x),
        static_cast<int>(b.position.y),
        config.separationRadius,
        GRAY
    );
}

void drawGridDebug(const Simulation& sim, GridDebugMode mode) {
    if (mode == GridDebugMode::None) {
        return;
    }

    const SpatialGrid& grid = sim.getGrid();
    const float cellSize = grid.getCellSize();

    for (const auto& [coord, indices] : grid.getCells()) {
        if (mode == GridDebugMode::HotCells && indices.size() < 4) {
            continue;
        }

        const int x = static_cast<int>(coord.x * cellSize);
        const int y = static_cast<int>(coord.y * cellSize);
        const int size = static_cast<int>(cellSize);

        DrawRectangleLines(x, y, size, size, DARKGREEN);

        if (indices.size() >= 2) {
            DrawText(
                TextFormat("%d", static_cast<int>(indices.size())),
                x + 4,
                y + 4,
                12,
                GREEN
            );
        }
    }
}

int drawCompactUi(
    bool paused,
    const Simulation& sim,
    const TunableParameter& selected,
    GridDebugMode gridDebugMode
) {
    const SimulationConfig& config = sim.getConfig();
    const SimulationStats stats = sim.getStats();

    int y = 10;
    const int lineHeight = 20;

    DrawText("boids_cpu", 10, y, 20, RAYWHITE);
    y += lineHeight + 6;

    DrawText(paused ? "Paused" : "Running", 10, y, 16, LIGHTGRAY);
    y += lineHeight;

    DrawText(
        TextFormat("Boids: %d", static_cast<int>(stats.boidCount)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Neighbor checks: %d", static_cast<int>(stats.neighborChecks)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Candidates: %d", static_cast<int>(stats.neighborCandidates)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Grid cells: %d", static_cast<int>(stats.occupiedGridCells)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        config.useSpatialGrid ? "Backend: spatial grid" : "Backend: naive",
        10,
        y,
        16,
        config.useSpatialGrid ? GREEN : LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Grid debug: %s", gridDebugModeName(gridDebugMode)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Selected: %s = %.2f", selected.name, *selected.value),
        10,
        y,
        16,
        YELLOW
    );
    y += lineHeight;

    DrawText("F1: UI mode", 10, y, 16, DARKGRAY);
    y += lineHeight;

    return y;
}

void drawFullUi(const SimulationConfig& config, int startY) {
    int y = startY;
    const int lineHeight = 20;

    DrawText(
        "Space: pause | N: step | R: reset | D: debug radii | F1: UI mode",
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        "Tab: select | Left/Right: adjust | Shift: fast",
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        "G: toggle grid backend | H: grid debug mode",
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat(
            "Align %.2f | Cohesion %.2f | Separation %.2f",
            config.alignmentWeight,
            config.cohesionWeight,
            config.separationWeight
        ),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat(
            "Perception %.1f | Separation radius %.1f | Grid cell %.1f",
            config.perceptionRadius,
            config.separationRadius,
            config.gridCellSize
        ),
        10,
        y,
        16,
        LIGHTGRAY
    );
}

}

int main() {
    InitWindow(WindowWidth, WindowHeight, "boids_cpu");
    SetTargetFPS(60);

    Simulation sim;

    bool paused = false;
    bool showDebug = false;
    UiMode uiMode = UiMode::Compact;
    GridDebugMode gridDebugMode = GridDebugMode::None;
    std::size_t selectedParameter = 0;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        auto& config = sim.getConfig();

        std::array<TunableParameter, 6> parameters{{
            {"alignmentWeight", &config.alignmentWeight, 0.0f, 0.5f, 2.0f},
            {"cohesionWeight", &config.cohesionWeight, 0.0f, 0.5f, 2.0f},
            {"separationWeight", &config.separationWeight, 0.0f, 0.5f, 2.0f},
            {"perceptionRadius", &config.perceptionRadius, 1.0f, 30.0f, 100.0f},
            {"separationRadius", &config.separationRadius, 1.0f, 30.0f, 100.0f},
            {"gridCellSize", &config.gridCellSize, 1.0f, 30.0f, 100.0f},
        }};

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }

        if (IsKeyPressed(KEY_R)) {
            sim.reset();
        }

        if (IsKeyPressed(KEY_D)) {
            showDebug = !showDebug;
        }

        if (IsKeyPressed(KEY_F1)) {
            uiMode = nextUiMode(uiMode);
        }

        if (IsKeyPressed(KEY_G)) {
            config.useSpatialGrid = !config.useSpatialGrid;
        }

        if (IsKeyPressed(KEY_H)) {
            gridDebugMode = nextGridDebugMode(gridDebugMode);
        }

        if (IsKeyPressed(KEY_TAB)) {
            selectedParameter = (selectedParameter + 1) % parameters.size();
        }

        const bool fastAdjust =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        TunableParameter& selected = parameters[selectedParameter];
        const float step = (fastAdjust ? selected.fastStep : selected.normalStep) * dt;

        if (IsKeyDown(KEY_LEFT)) {
            *selected.value = clampMin(*selected.value - step, selected.minValue);
        }

        if (IsKeyDown(KEY_RIGHT)) {
            *selected.value = clampMin(*selected.value + step, selected.minValue);
        }

        const bool stepFrame = paused && IsKeyPressed(KEY_N);

        if (!paused || stepFrame) {
            sim.update(dt);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        drawGridDebug(sim, gridDebugMode);

        for (const auto& b : sim.getBoids()) {
            if (showDebug) {
                drawDebugRadii(b, config);
            }

            drawBoid(b);
        }

        if (uiMode != UiMode::None) {
            const int nextY = drawCompactUi(paused, sim, selected, gridDebugMode);

            if (uiMode == UiMode::Full) {
                drawFullUi(config, nextY + 10);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}