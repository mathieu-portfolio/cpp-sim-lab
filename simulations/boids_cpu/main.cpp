#include "Simulation.hpp"

#include <raylib.h>

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;

Vector2 toRaylib(Vec2 v) {
    return Vector2{v.x, v.y};
}
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "boids_cpu");
    SetTargetFPS(60);

    Simulation sim;

    bool paused = false;
    bool showDebug = false;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }

        if (IsKeyPressed(KEY_R)) {
            sim.reset();
        }

        if (IsKeyPressed(KEY_D)) {
            showDebug = !showDebug;
        }

        const bool step = paused && IsKeyPressed(KEY_N);

        if (!paused || step) {
            sim.update(dt);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        const auto& config = sim.getConfig();

        for (const auto& b : sim.getBoids()) {
            if (showDebug) {
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

            const Vec2 dir = b.velocity.normalized();
            const Vec2 tip = b.position + dir * 8.0f;
            const Vec2 left = b.position + Vec2{-dir.y, dir.x} * 4.0f;
            const Vec2 right = b.position + Vec2{dir.y, -dir.x} * 4.0f;

            DrawTriangle(
                toRaylib(tip),
                toRaylib(left),
                toRaylib(right),
                WHITE
            );
        }

        DrawText("boids_cpu", 10, 10, 20, RAYWHITE);
        DrawText(paused ? "State: paused" : "State: running", 10, 36, 16, LIGHTGRAY);
        DrawText("Space: pause | N: step | R: reset | D: debug", 10, 58, 16, LIGHTGRAY);

        DrawText(
            TextFormat("Boids: %d", static_cast<int>(sim.getBoids().size())),
            10,
            82,
            16,
            LIGHTGRAY
        );

        DrawText(
            TextFormat(
                "Perception: %.1f | Separation: %.1f",
                config.perceptionRadius,
                config.separationRadius
            ),
            10,
            104,
            16,
            LIGHTGRAY
        );

        EndDrawing();
    }

    CloseWindow();
    return 0;
}