#include "Simulation.hpp"
#include "RoadBrush.hpp"
#include "SimulationUiTraits.hpp"

#include <raylib.h>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationFrameHelpers.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include <ui/UiHelpers.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/RaylibCamera.hpp>

#include <algorithm>
#include <tuple>
#include <ui/EntitySelection.hpp>

using namespace traffic_flow_cpu;

namespace {
constexpr int WindowWidth = 1100;
constexpr int WindowHeight = 700;
constexpr float MinRoadLength = 100.0f;
constexpr float MaxRoadLength = 4000.0f;
constexpr float RoadBrushRadius = 18.0f;
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "traffic_flow_cpu");
    SetTargetFPS(60);

    Simulation sim;
    simfw::ui::SimulationControls controls;
    Camera2D camera = simfw::ui::makeCenteredCamera(
        static_cast<float>(WindowWidth),
        static_cast<float>(WindowHeight)
    );
    RoadBrush roadBrush;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& config = sim.getConfig();
        constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);
        simfw::ui::handleTunableAdjustment(config, controls, dt);

        simfw::ui::zoomCameraAtScreenPoint(
            camera,
            GetMousePosition(),
            GetMouseWheelMove(),
            0.1f,
            0.5f,
            8.0f,
            static_cast<float>(WindowWidth),
            static_cast<float>(WindowHeight)
        );

        if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
            simfw::ui::panCameraByScreenDelta(
                camera,
                GetMouseDelta(),
                static_cast<float>(WindowWidth),
                static_cast<float>(WindowHeight)
            );
        }

        if (IsKeyPressed(KEY_BACKSPACE)) {
            simfw::ui::resetCameraToBounds(
                camera,
                static_cast<float>(WindowWidth),
                static_cast<float>(WindowHeight)
            );
        }

        {
            const Vec2 mouseWorld = simfw::ui::screenToWorld(GetMousePosition(), camera);
            const float roadX = 80.0f;
            const float roadW = 920.0f;
            const float roadPixelsPerSimUnit = roadW / std::max(1.0f, config.roadLength);
            const bool roadChanged = roadBrush.paint(
                IsMouseButtonDown(MOUSE_RIGHT_BUTTON),
                mouseWorld,
                RoadBrushRadius / camera.zoom,
                roadX,
                roadPixelsPerSimUnit,
                MinRoadLength,
                MaxRoadLength,
                config.roadLength
            );
            (void)roadChanged;
        }

        if (simfw::ui::shouldAdvanceSimulation(controls)) {
            sim.update(dt);
            simfw::ui::finishSimulationStep(controls);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        const auto& vehicles = sim.getVehicles();
        const auto& stats = sim.getStats();

        const float roadX = 80.0f;
        const float roadY = 150.0f;
        const float roadW = 920.0f;
        const float roadH = config.laneWidth * static_cast<float>(config.laneCount);

        BeginMode2D(camera);

        DrawRectangleLinesEx({roadX, roadY, roadW, roadH}, 2.0f, DARKGRAY);
        for (int lane = 1; lane < config.laneCount; ++lane) {
            const float y = roadY + config.laneWidth * static_cast<float>(lane);
            DrawLineEx({roadX, y}, {roadX + roadW, y}, 1.0f, GRAY);
        }

        for (const Vehicle& v : vehicles) {
            const float x = roadX + (v.s / config.roadLength) * roadW;
            const float y = roadY + (static_cast<float>(v.lane) + 0.5f) * config.laneWidth;
            const Color color = v.speed < 3.0f ? ORANGE : SKYBLUE;
            DrawRectangleV({x - 6.0f, y - 5.0f}, {12.0f, 10.0f}, color);
        }

        EndMode2D();

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{30, 24, 20};
            cursor.draw("traffic_flow_cpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");
            cursor.draw(TextFormat("Vehicles: %d", static_cast<int>(vehicles.size())));
            simfw::ui::drawStats(cursor, stats);
            cursor.gap(6);
            simfw::ui::drawTunables(cursor, config, controls.selectedParameter);
            cursor.gap(8);
            cursor.draw(TextFormat("Zoom: %.2fx", camera.zoom), 18, GRAY);
        }

        if (controls.uiMode == simfw::ui::UiMode::Full) {
            simfw::ui::TextCursor controlsCursor =
                simfw::ui::makeRightSideControlCursor(340, 10, 20);

            simfw::ui::drawControlHints(
                controlsCursor,
                {
                    "Space: pause",
                    "N: step",
                    "R: reset",
                    "F1: UI mode",
                    "Wheel: zoom",
                    "Middle mouse: pan",
                    "Right mouse: brush road extension",
                    "Backspace: reset camera",
                    "Tab: select tunable",
                    "Left/Right: adjust",
                    "Shift: fast adjust"
                }
            );
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
