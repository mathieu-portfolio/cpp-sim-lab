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

using namespace traffic_flow_cpu_gpu;

namespace {
constexpr int WindowWidth = 1100;
constexpr int WindowHeight = 700;
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "traffic_flow_cpu_gpu");
    SetTargetFPS(60);

    Simulation sim;
    simfw::ui::SimulationControls controls;
    RoadBrush roadBrush;
    Camera2D camera = simfw::ui::makeCenteredCamera(
        static_cast<float>(WindowWidth),
        static_cast<float>(WindowHeight)
    );

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

        if (IsKeyPressed(KEY_G)) {
            sim.generateTraffic();
        }

        const Vector2 mouseScreen = GetMousePosition();
        const Vector2 mouseWorldRaylib = GetScreenToWorld2D(mouseScreen, camera);
        const Vec2 mouseWorld{mouseWorldRaylib.x, mouseWorldRaylib.y};
        constexpr float roadBrushSpacing = 28.0f;
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            auto& road = sim.getRoadNetwork().roads[0];
            road.controlPoints.clear();
        }
        if (roadBrush.paint(IsMouseButtonDown(MOUSE_LEFT_BUTTON), mouseWorld, roadBrushSpacing)) {
            auto& road = sim.getRoadNetwork().roads[0];
            road.controlPoints.push_back(mouseWorld);
            sim.notifyRoadsEdited();
        }

        if (simfw::ui::shouldAdvanceSimulation(controls)) {
            sim.update(dt);
            simfw::ui::finishSimulationStep(controls);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        const auto& vehicles = sim.getVehicles();
        const auto& stats = sim.getStats();

        BeginMode2D(camera);
        const auto& road = sim.getRoadNetwork().roads[0];
        if (road.controlPoints.size() >= 2 && road.length > 0.0f) {
            constexpr int roadDrawSamples = 96;
            Vec2 previous = sim.sampleRoadCenter(0, 0.0f);
            for (int i = 1; i <= roadDrawSamples; ++i) {
                const float s = road.length * static_cast<float>(i) / static_cast<float>(roadDrawSamples);
                const Vec2 current = sim.sampleRoadCenter(0, s);
                DrawLineEx({previous.x, previous.y}, {current.x, current.y}, config.laneWidth * 2.0f, DARKGRAY);
                DrawLineEx({previous.x, previous.y}, {current.x, current.y}, 2.0f, YELLOW);
                previous = current;
            }
        }
        for (const auto& crossroad : sim.getRoadNetwork().crossroads) {
            DrawCircleV({crossroad.position.x, crossroad.position.y}, 8.0f, RED);
            DrawCircleLines(static_cast<int>(crossroad.position.x), static_cast<int>(crossroad.position.y), config.crossroadYieldLookahead, MAROON);
        }

        for (const Vehicle& v : vehicles) {
            const Vec2 p = sim.sampleLanePosition(v.roadId, v.laneId, v.s);
            const Vector2 pos{p.x,p.y};
            const Color color = v.speed < 3.0f ? ORANGE : SKYBLUE;
            DrawRectangleV({pos.x - 6.0f, pos.y - 5.0f}, {12.0f, 10.0f}, color);
        }

        EndMode2D();

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{30, 24, 20};
            cursor.draw("traffic_flow_cpu_gpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");
            cursor.draw(TextFormat("Vehicles: %d", static_cast<int>(vehicles.size())));
            cursor.draw(TextFormat("Crossroads: %d", static_cast<int>(sim.getRoadNetwork().crossroads.size())));
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
                    "G: generate traffic",
                    "Left mouse: paint road",
                    "F1: UI mode",
                    "Wheel: zoom",
                    "Middle mouse: pan",
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
