#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include "raylib.h"
#include <algorithm>

void clampCameraToBounds(Camera2D& camera, const SimulationConfig& config) {
    const float viewWidth = config.width / camera.zoom;
    const float viewHeight = config.height / camera.zoom;

    if (viewWidth >= config.width) {
        camera.target.x = config.width * 0.5f;
    } else {
        const float halfViewWidth = viewWidth * 0.5f;
        camera.target.x = std::clamp(
            camera.target.x,
            halfViewWidth,
            config.width - halfViewWidth
        );
    }

    if (viewHeight >= config.height) {
        camera.target.y = config.height * 0.5f;
    } else {
        const float halfViewHeight = viewHeight * 0.5f;
        camera.target.y = std::clamp(
            camera.target.y,
            halfViewHeight,
            config.height - halfViewHeight
        );
    }
}

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

    Camera2D camera{};
    camera.target = Vector2{config.width * 0.5f, config.height * 0.5f};
    camera.offset = Vector2{config.width * 0.5f, config.height * 0.5f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;

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

        const float wheel = GetMouseWheelMove();

        if (wheel != 0.0f) {
            const Vector2 mouseWorldBefore = GetScreenToWorld2D(GetMousePosition(), camera);

            camera.zoom *= 1.0f + wheel * 0.1f;
            camera.zoom = std::clamp(camera.zoom, 1.0f, 8.0f);

            const Vector2 mouseWorldAfter = GetScreenToWorld2D(GetMousePosition(), camera);

            camera.target.x += mouseWorldBefore.x - mouseWorldAfter.x;
            camera.target.y += mouseWorldBefore.y - mouseWorldAfter.y;

            clampCameraToBounds(camera, config);
        }

        if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
            const Vector2 delta = GetMouseDelta();

            camera.target.x -= delta.x / camera.zoom;
            camera.target.y -= delta.y / camera.zoom;

            clampCameraToBounds(camera, config);
        }

        if (IsKeyPressed(KEY_BACKSPACE)) {
            camera.target = Vector2{config.width * 0.5f, config.height * 0.5f};
            camera.offset = Vector2{config.width * 0.5f, config.height * 0.5f};
            camera.zoom = 1.0f;
        }

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode2D(camera);

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

        EndMode2D();

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
        DrawText("Mouse wheel: zoom", 10, y, 18, GRAY); y += line;
        DrawText("Middle mouse: pan", 10, y, 18, GRAY); y += line;
        DrawText("Backspace: reset camera", 10, y, 18, GRAY); y += line;

        // --- State ---
        y += 8;
        DrawText(paused ? "Paused" : "Running", 10, y, 18, YELLOW);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
