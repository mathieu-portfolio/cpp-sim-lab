#include "Simulation.hpp"

#include <raylib.h>

int main() {
    InitWindow(800, 800, "boids_cpu");
    SetTargetFPS(60);

    Simulation simulation;

    while (!WindowShouldClose()) {
        simulation.update(GetFrameTime());

        BeginDrawing();
        ClearBackground(BLACK);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
