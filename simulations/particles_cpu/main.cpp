#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>

#include <raylib.h>

namespace {
void drawCompactUi(
    bool paused,
    const Simulation& sim,
    const Camera2D& camera
) {
    const auto stats = sim.getStats();

    simfw::ui::TextCursor cursor{10, 10, 22};

    cursor.draw(
        TextFormat(
            "Particles: %d / %d",
            static_cast<int>(stats.particleCount),
            static_cast<int>(stats.maxParticleCount)
        ),
        20,
        GREEN
    );

    cursor.draw(
        TextFormat(
            "Collision checks: %d",
            static_cast<int>(stats.collisionChecks)
        ),
        18,
        GRAY
    );

    cursor.draw(
        TextFormat(
            "Collisions resolved: %d",
            static_cast<int>(stats.collisionsResolved)
        ),
        18,
        GRAY
    );

    cursor.gap(12);

    cursor.draw("Left mouse: spawn", 18, GRAY);
    cursor.draw("Right mouse: clear", 18, GRAY);
    cursor.draw("R: reset", 18, GRAY);
    cursor.draw("Space: pause", 18, GRAY);
    cursor.draw("N: step", 18, GRAY);
    cursor.draw("G: toggle grid", 18, GRAY);
    cursor.draw("Mouse wheel: zoom", 18, GRAY);
    cursor.draw("Middle mouse: pan", 18, GRAY);
    cursor.draw("Backspace: reset camera", 18, GRAY);

    cursor.draw(
        TextFormat("Zoom: %.2fx", camera.zoom),
        18,
        GRAY
    );

    cursor.gap(8);
    cursor.draw(paused ? "Paused" : "Running", 18, YELLOW);
}
} // namespace

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
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;

    Camera2D camera = simfw::ui::makeCenteredCamera(
        config.width,
        config.height
    );

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        if (IsKeyPressed(KEY_SPACE)) {
            paused = !paused;
        }

        if (IsKeyPressed(KEY_G)) {
            gridDebugMode = simfw::ui::nextGridDebugMode(gridDebugMode);
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

        simfw::ui::zoomCameraAtScreenPoint(
            camera,
            GetMousePosition(),
            GetMouseWheelMove(),
            0.1f,
            1.0f,
            8.0f,
            config.width,
            config.height
        );

        if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
            simfw::ui::panCameraByScreenDelta(
                camera,
                GetMouseDelta(),
                config.width,
                config.height
            );
        }

        if (IsKeyPressed(KEY_BACKSPACE)) {
            simfw::ui::resetCameraToBounds(
                camera,
                config.width,
                config.height
            );
        }

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode2D(camera);

        simfw::ui::drawSpatialGridDebug(
            sim.getGrid(),
            gridDebugMode,
            2,
            YELLOW,
            ORANGE
        );

        for (const auto& p : sim.getParticles()) {
            DrawCircle(
                static_cast<int>(p.position.x),
                static_cast<int>(p.position.y),
                p.radius,
                WHITE
            );
        }

        EndMode2D();

        drawCompactUi(paused, sim, camera);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
