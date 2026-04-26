#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include "raylib.h"

int main() {
    const int width = 800;
    const int height = 800;

    InitWindow(width, height, "particles_cpu");
    SetTargetFPS(60);

    Simulation sim;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // input
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
            DrawCircle((int)p.position.x, (int)p.position.y, 4, WHITE);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
