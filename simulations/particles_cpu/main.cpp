#include "Particle.hpp"

#include <math/Vec2.hpp>
#include <random/Random.hpp>
#include <time/FixedTimestep.hpp>

#include "raylib.h"

#include <vector>

static void integrate(std::vector<Particle>& particles, float dt) {
    const Vec2 gravity{0.0f, 200.0f}; // scaled for screen

    for (auto& p : particles) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;

        if (p.position.y > 800) {
            p.position.y = 800;
            p.velocity.y *= -0.8f;
        }
    }
}

int main() {
    const int width = 800;
    const int height = 800;

    InitWindow(width, height, "particles_cpu");
    SetTargetFPS(60);

    std::vector<Particle> particles;

    for (int i = 0; i < 100; ++i) {
        particles.push_back({
            Vec2{Random::range(0, width), Random::range(0, height)},
            Vec2{Random::range(-50, 50), Random::range(-50, 50)}
        });
    }

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        integrate(particles, dt);

        BeginDrawing();
        ClearBackground(BLACK);

        for (auto& p : particles) {
            DrawCircle((int)p.position.x, (int)p.position.y, 4, WHITE);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}