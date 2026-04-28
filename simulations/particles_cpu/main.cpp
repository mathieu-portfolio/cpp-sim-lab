#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include "SimulationUiTraits.hpp"

#include <raylib.h>

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
    std::size_t selectedParameter = 0;

    Camera2D camera = simfw::ui::makeCenteredCamera(
        config.width,
        config.height
    );

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        constexpr std::size_t paramCount =
            std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;

        selectedParameter %= paramCount;

        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_G)) gridDebugMode = simfw::ui::nextGridDebugMode(gridDebugMode);
        if (IsKeyPressed(KEY_N)) step = true;
        if (IsKeyPressed(KEY_R)) sim.reset();

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            const auto mouse = GetMousePosition();
            sim.spawn(Vec2{mouse.x, mouse.y});
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) sim.clear();

        const bool fastAdjust =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        if (IsKeyDown(KEY_LEFT)) {
            simfw::ui::adjustTunables(config, selectedParameter, -1.0f, dt, fastAdjust);
        }

        if (IsKeyDown(KEY_RIGHT)) {
            simfw::ui::adjustTunables(config, selectedParameter, 1.0f, dt, fastAdjust);
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

        simfw::ui::TextCursor cursor{10, 10, 22};

        simfw::ui::drawStats(cursor, sim.getStats());
        cursor.gap(6);
        simfw::ui::drawTunables(cursor, config, selectedParameter);

        cursor.gap(10);
        cursor.draw("Space: pause | N: step | R: reset");
        cursor.draw("Mouse: spawn | Right: clear | Wheel: zoom | Middle: pan");

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
