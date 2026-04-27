#include "Simulation.hpp"
#include <raylib.h>

int main() {
    InitWindow(800, 800, "boids_cpu");
    SetTargetFPS(60);

    Simulation sim;

    while (!WindowShouldClose()) {
        sim.update(GetFrameTime());

        BeginDrawing();
        ClearBackground(BLACK);

        for (const auto& b : sim.getBoids()) {
            DrawCircleV({b.position.x, b.position.y}, 3.0f, WHITE);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
