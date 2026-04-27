#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include "raylib.h"

int main() {
    SimulationConfig config;
    config.width = 800.0f;
    config.height = 800.0f;
    config.maxParticleCount = 1000;

    InitWindow(
        static_cast<int>(config.width),
        static_cast<int>(config.height),
        "particles_cpu"
    );

    SetTargetFPS(60);

    Simulation sim{config};
    sim.reset();

    bool paused = false;
    bool step = false;
    bool showGrid = false;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }

        if (IsKeyPressed(KEY_G)) {
            showGrid = !showGrid;
        }

        if (IsKeyPressed(KEY_N)) {
            step = true;
        }

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            const auto mouse = GetMousePosition();
            sim.spawn(Vec2{mouse.x, mouse.y});
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            sim.clear();
        }

        if (IsKeyPressed(KEY_R)) {
            sim.reset();
        }

        if (!paused || step) {
            sim.update(dt);
            step = false;
        }

        BeginDrawing();
        ClearBackground(BLACK);

        if (showGrid) {
            const auto& config = sim.getConfig();
            const auto& cells = sim.getGrid().cells();

            for (const auto& [coord, indices] : cells) {
                const int x = static_cast<int>(coord.x * config.gridCellSize);
                const int y = static_cast<int>(coord.y * config.gridCellSize);
                const int size = static_cast<int>(config.gridCellSize);

                DrawRectangleLines(x, y, size, size, YELLOW);

                if (indices.size() > 1) {
                    DrawText(
                        TextFormat("%d", static_cast<int>(indices.size())),
                        x + 2,
                        y + 2,
                        10,
                        ORANGE
                    );
                }
            }
        }

        for (const auto& p : sim.getParticles()) {
            DrawCircle(
                static_cast<int>(p.position.x),
                static_cast<int>(p.position.y),
                p.radius,
                WHITE
            );
        }

        const auto stats = sim.getStats();

        int y = 10;
        const int line = 22;

        // --- Stats ---
        DrawText(
            TextFormat("Particles: %d / %d",
                static_cast<int>(stats.particleCount),
                static_cast<int>(stats.maxParticleCount)),
            10, y, 20, GREEN
        );
        y += line;

        DrawText(
            TextFormat("Collision checks: %d", static_cast<int>(stats.collisionChecks)),
            10, y, 18, GRAY
        );
        y += line;

        DrawText(
            TextFormat("Collisions resolved: %d", static_cast<int>(stats.collisionsResolved)),
            10, y, 18, GRAY
        );
        y += line + 12;

        // --- Controls ---
        DrawText("Left mouse: spawn", 10, y, 18, GRAY); y += line;
        DrawText("Right mouse: clear", 10, y, 18, GRAY); y += line;
        DrawText("R: reset", 10, y, 18, GRAY); y += line;
        DrawText("Space: pause", 10, y, 18, GRAY); y += line;
        DrawText("N: step", 10, y, 18, GRAY); y += line;
        DrawText("G: toggle grid", 10, y, 18, GRAY); y += line;

        // --- State ---
        y += 8;
        DrawText(paused ? "Paused" : "Running", 10, y, 18, YELLOW);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
