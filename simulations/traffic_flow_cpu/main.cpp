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

Vector2 roadCenterPoint(float t, float roadX, float roadY, float roadW, float roadH) {
    const float clampedT = std::clamp(t, 0.0f, 1.0f);
    const float x = roadX + clampedT * roadW;
    const float centerY = roadY + roadH * 0.5f;
    const float amplitude = roadH * 0.9f;
    const float y = centerY + std::sin(clampedT * PI * 2.2f) * amplitude;
    return {x, y};
}

Vector2 roadPoint(float t, float lateralOffset, float roadX, float roadY, float roadW, float roadH) {
    constexpr float tangentStep = 0.0015f;
    const Vector2 center = roadCenterPoint(t, roadX, roadY, roadW, roadH);
    const Vector2 prev = roadCenterPoint(t - tangentStep, roadX, roadY, roadW, roadH);
    const Vector2 next = roadCenterPoint(t + tangentStep, roadX, roadY, roadW, roadH);
    Vector2 tangent{next.x - prev.x, next.y - prev.y};
    const float tangentLength = std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y);
    if (tangentLength < 1e-4f) {
        tangent = {1.0f, 0.0f};
    } else {
        tangent.x /= tangentLength;
        tangent.y /= tangentLength;
    }

    const Vector2 normal{-tangent.y, tangent.x};
    return {center.x + normal.x * lateralOffset, center.y + normal.y * lateralOffset};
}
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

        const int roadSamples = std::max(32, static_cast<int>(roadW / 8.0f));
        const float halfRoadWidth = roadH * 0.5f;

        for (int i = 1; i <= roadSamples; ++i) {
            const float t0 = static_cast<float>(i - 1) / static_cast<float>(roadSamples);
            const float t1 = static_cast<float>(i) / static_cast<float>(roadSamples);
            const Vector2 left0 = roadPoint(t0, -halfRoadWidth, roadX, roadY, roadW, roadH);
            const Vector2 left1 = roadPoint(t1, -halfRoadWidth, roadX, roadY, roadW, roadH);
            const Vector2 right0 = roadPoint(t0, halfRoadWidth, roadX, roadY, roadW, roadH);
            const Vector2 right1 = roadPoint(t1, halfRoadWidth, roadX, roadY, roadW, roadH);
            DrawLineEx(left0, left1, 2.0f, DARKGRAY);
            DrawLineEx(right0, right1, 2.0f, DARKGRAY);
        }

        for (int lane = 1; lane < config.laneCount; ++lane) {
            const float offset = -halfRoadWidth + config.laneWidth * static_cast<float>(lane);
            for (int i = 1; i <= roadSamples; ++i) {
                const float t0 = static_cast<float>(i - 1) / static_cast<float>(roadSamples);
                const float t1 = static_cast<float>(i) / static_cast<float>(roadSamples);
                DrawLineEx(
                    roadPoint(t0, offset, roadX, roadY, roadW, roadH),
                    roadPoint(t1, offset, roadX, roadY, roadW, roadH),
                    1.0f,
                    GRAY
                );
            }
        }

        for (const Vehicle& v : vehicles) {
            const float t = v.s / std::max(1.0f, config.roadLength);
            const float laneOffset = -halfRoadWidth + (static_cast<float>(v.lane) + 0.5f) * config.laneWidth;
            const Vector2 pos = roadPoint(t, laneOffset, roadX, roadY, roadW, roadH);
            const Color color = v.speed < 3.0f ? ORANGE : SKYBLUE;
            DrawRectangleV({pos.x - 6.0f, pos.y - 5.0f}, {12.0f, 10.0f}, color);
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
