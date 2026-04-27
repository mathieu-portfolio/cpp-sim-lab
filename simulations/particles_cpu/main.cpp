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

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
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

        for (const auto& p : sim.getParticles()) {
            DrawCircle(
                static_cast<int>(p.position.x),
                static_cast<int>(p.position.y),
                p.radius,
                WHITE
            );
        }

        const auto stats = sim.getStats();

        DrawText(
            TextFormat("Particles: %d / %d",
                static_cast<int>(stats.particleCount),
                static_cast<int>(stats.maxParticleCount)),
            10,
            10,
            20,
            GREEN
        );

        DrawText(
            TextFormat("Collision checks: %d", static_cast<int>(stats.collisionChecks)),
            10,
            34,
            18,
            GRAY
        );

        DrawText(
            TextFormat("Collisions resolved: %d", static_cast<int>(stats.collisionsResolved)),
            10,
            56,
            18,
            GRAY
        );

        DrawText("Left mouse: spawn", 10, 90, 18, GRAY);
        DrawText("Right mouse: clear", 10, 112, 18, GRAY);
        DrawText("R: reset", 10, 134, 18, GRAY);
        DrawText("Space: pause", 10, 156, 18, GRAY);
        DrawText("N: step", 10, 178, 18, GRAY);
        DrawText(paused ? "Paused" : "Running", 10, 200, 18, YELLOW);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
