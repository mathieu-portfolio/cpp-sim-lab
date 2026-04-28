#include "Simulation.hpp"

#include <array>
#include <ui/RaylibDebugUi.hpp>

#include <raylib.h>

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;

void drawBoid(const Boid& b) {
    DrawCircleV(simfw::ui::toRaylib(b.position), 3.0f, WHITE);

    Vec2 dir = b.velocity.length() > 0.001f
        ? b.velocity.normalized()
        : Vec2{1.0f, 0.0f};

    Vec2 end = b.position + dir * 10.0f;

    DrawLineV(
        simfw::ui::toRaylib(b.position),
        simfw::ui::toRaylib(end),
        GRAY
    );
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

int drawCompactUi(
    bool paused,
    const Simulation& sim,
    const simfw::ui::TunableParameter& selected,
    simfw::ui::GridDebugMode gridDebugMode
) {
    const SimulationConfig& config = sim.getConfig();
    const SimulationStats stats = sim.getStats();

    simfw::ui::TextCursor cursor{10, 10, 20};

    cursor.draw("boids_cpu", 20, RAYWHITE);
    cursor.gap(6);

    cursor.draw(paused ? "Paused" : "Running");

    cursor.draw(
        TextFormat("Boids: %d", static_cast<int>(stats.boidCount))
    );

    cursor.draw(
        TextFormat("Neighbor checks: %d", static_cast<int>(stats.neighborChecks))
    );

    cursor.draw(
        TextFormat("Candidates: %d", static_cast<int>(stats.neighborCandidates))
    );

    cursor.draw(
        TextFormat("Grid cells: %d", static_cast<int>(stats.occupiedGridCells))
    );

    cursor.draw(
        config.useSpatialGrid ? "Backend: spatial grid" : "Backend: naive",
        16,
        config.useSpatialGrid ? GREEN : LIGHTGRAY
    );

    cursor.draw(
        TextFormat(
            "Grid debug: %s",
            simfw::ui::gridDebugModeName(gridDebugMode)
        )
    );

    cursor.draw(
        TextFormat("Selected: %s = %.2f", selected.name, *selected.value),
        16,
        YELLOW
    );

    cursor.draw("F1: UI mode", 16, DARKGRAY);

    return cursor.y;
}

void drawFullUi(const SimulationConfig& config, int startY) {
    simfw::ui::TextCursor cursor{10, startY, 20};

    cursor.draw(
        "Space: pause | N: step | R: reset | D: debug radii | F1: UI mode"
    );

    cursor.draw("Tab: select | Left/Right: adjust | Shift: fast");

    cursor.draw("G: toggle grid backend | H: grid debug mode");

    cursor.draw(
        TextFormat(
            "Align %.2f | Cohesion %.2f | Separation %.2f",
            config.alignmentWeight,
            config.cohesionWeight,
            config.separationWeight
        )
    );

    cursor.draw(
        TextFormat(
            "Perception %.1f | Separation radius %.1f | Grid cell %.1f",
            config.perceptionRadius,
            config.separationRadius,
            config.gridCellSize
        )
    );
}

} // namespace

int main() {
    InitWindow(WindowWidth, WindowHeight, "boids_cpu");
    SetTargetFPS(60);

    Simulation sim;

    bool paused = false;
    bool showDebug = false;
    simfw::ui::UiMode uiMode = simfw::ui::UiMode::Compact;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;
    std::size_t selectedParameter = 0;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        auto& config = sim.getConfig();

        std::array<simfw::ui::TunableParameter, 6> parameters{{
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
            uiMode = simfw::ui::nextUiMode(uiMode);
        }

        if (IsKeyPressed(KEY_G)) {
            config.useSpatialGrid = !config.useSpatialGrid;
        }

        if (IsKeyPressed(KEY_H)) {
            gridDebugMode = simfw::ui::nextGridDebugMode(gridDebugMode);
        }

        if (IsKeyPressed(KEY_TAB)) {
            selectedParameter = (selectedParameter + 1) % parameters.size();
        }

        const bool fastAdjust =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        simfw::ui::TunableParameter& selected = parameters[selectedParameter];

        if (IsKeyDown(KEY_LEFT)) {
            simfw::ui::adjustTunable(selected, -1.0f, dt, fastAdjust);
        }

        if (IsKeyDown(KEY_RIGHT)) {
            simfw::ui::adjustTunable(selected, 1.0f, dt, fastAdjust);
        }

        const bool stepFrame = paused && IsKeyPressed(KEY_N);

        if (!paused || stepFrame) {
            sim.update(dt);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode);

        for (const auto& b : sim.getBoids()) {
            if (showDebug) {
                drawDebugRadii(b, config);
            }

            drawBoid(b);
        }

        if (uiMode != simfw::ui::UiMode::None) {
            const int nextY = drawCompactUi(
                paused,
                sim,
                selected,
                gridDebugMode
            );

            if (uiMode == simfw::ui::UiMode::Full) {
                drawFullUi(config, nextY + 10);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
