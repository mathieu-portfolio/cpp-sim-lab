#include "Simulation.hpp"

#include <math/Vec2.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationControls.hpp>
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

    simfw::ui::SimulationControls controls;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;

    Camera2D camera = simfw::ui::makeCenteredCamera(
        config.width,
        config.height
    );

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        constexpr std::size_t paramCount =
            std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;

        simfw::ui::handleCommonSimulationControls(
            controls,
            sim,
            paramCount
        );

        if (IsKeyPressed(KEY_G)) {
            gridDebugMode = simfw::ui::nextGridDebugMode(gridDebugMode);
        }

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            const auto mouse = GetMousePosition();
            sim.spawn(Vec2{mouse.x, mouse.y});
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            sim.clear();
        }

        const bool fastAdjust =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        if (IsKeyDown(KEY_LEFT)) {
            simfw::ui::adjustTunables(
                config,
                controls.selectedParameter,
                -1.0f,
                dt,
                fastAdjust
            );
        }

        if (IsKeyDown(KEY_RIGHT)) {
            simfw::ui::adjustTunables(
                config,
                controls.selectedParameter,
                1.0f,
                dt,
                fastAdjust
            );
        }

        if (simfw::ui::shouldAdvanceSimulation(controls)) {
            sim.update(dt);
            simfw::ui::finishSimulationStep(controls);
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

        if (controls.showDebug) {
            simfw::ui::drawSpatialGridDebug(
                sim.getGrid(),
                gridDebugMode,
                2,
                YELLOW,
                ORANGE
            );
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

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 22};

            cursor.draw("particles_cpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");

            simfw::ui::drawStats(cursor, sim.getStats());

            cursor.gap(6);

            simfw::ui::drawTunables(
                cursor,
                config,
                controls.selectedParameter
            );

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                cursor.gap(10);
                cursor.draw("Space: pause | N: step | R: reset | D: debug grid | F1: UI mode");
                cursor.draw("Tab: select | Left/Right: adjust | Shift: fast");
                cursor.draw("Mouse: spawn | Right: clear | Wheel: zoom | Middle: pan");
                cursor.draw("G: grid debug mode | Backspace: reset camera");
            }

            cursor.gap(8);
            cursor.draw(
                TextFormat("Zoom: %.2fx", camera.zoom),
                18,
                GRAY
            );
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
