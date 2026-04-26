#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include "raylib.h"

int main() {
    const int width = 800;
    const int height = 800;

    InitWindow(width, height, "particles_cpu");
    SetTargetFPS(60);

    Simulation sim{5000};

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            auto mouse = GetMousePosition();
            sim.spawn(Vec2{mouse.x, mouse.y});
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            sim.clear();
        }

        sim.update(dt);

        BeginDrawing();
        ClearBackground(BLACK);

        for (const auto& p : sim.getParticles()) {
            DrawCircle(static_cast<int>(p.position.x), static_cast<int>(p.position.y), 4, WHITE);
        }

        auto stats = sim.getStats();
        DrawText(
            TextFormat("Particles: %d / %d",
                static_cast<int>(stats.particleCount),
                static_cast<int>(stats.maxParticleCount)),
            10,
            10,
            20,
            GREEN
        );

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
