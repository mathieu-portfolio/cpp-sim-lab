#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <ui/EntitySelection.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/SpatialGridDebugDraw.hpp>
#include <ui/UiHelpers.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <raylib.h>

#include <optional>

using namespace bubbles;

namespace {
constexpr float SelectionPickRadius = 20.0f;
constexpr float SelectionRingPadding = 4.0f;

Vec2 bubblePosition(const Bubble& bubble) {
    return bubble.position;
}
} // namespace

int main() {
    SimulationConfig config;
    InitWindow(static_cast<int>(config.width), static_cast<int>(config.height), "bubbles");
    SetTargetFPS(60);

    Simulation sim{config};
    sim.reset();

    simfw::ui::SimulationControls controls;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;
    std::optional<std::size_t> selectedBubble;
    Camera2D camera = simfw::ui::makeCenteredCamera(config.width, config.height);

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& liveConfig = sim.getConfig();

        constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);
        simfw::ui::handleSimulationBackendControls(liveConfig, gridDebugMode);

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            sim.spawn(simfw::ui::screenToWorld(GetMousePosition(), camera));
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            selectedBubble = simfw::ui::findClosestEntity(
                sim.getBubbles(),
                simfw::ui::screenToWorld(GetMousePosition(), camera),
                SelectionPickRadius / camera.zoom,
                bubblePosition
            );
        }

        if (IsKeyPressed(KEY_C)) {
            sim.clear();
            selectedBubble.reset();
        }

        if (IsKeyPressed(KEY_B)) {
            liveConfig.enableBurst = !liveConfig.enableBurst;
        }

        const bool fastAdjust = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        if (IsKeyDown(KEY_LEFT)) {
            simfw::ui::adjustTunables(liveConfig, controls.selectedParameter, -1.0f, dt, fastAdjust);
        }
        if (IsKeyDown(KEY_RIGHT)) {
            simfw::ui::adjustTunables(liveConfig, controls.selectedParameter, 1.0f, dt, fastAdjust);
        }

        if (simfw::ui::shouldAdvanceSimulation(controls)) {
            sim.update(dt);
            simfw::ui::finishSimulationStep(controls);
        }

        simfw::ui::zoomCameraAtScreenPoint(camera, GetMousePosition(), GetMouseWheelMove(), 0.1f, 1.0f, 8.0f, liveConfig.width, liveConfig.height);
        if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
            simfw::ui::panCameraByScreenDelta(camera, GetMouseDelta(), liveConfig.width, liveConfig.height);
        }
        if (IsKeyPressed(KEY_BACKSPACE)) {
            simfw::ui::resetCameraToBounds(camera, liveConfig.width, liveConfig.height);
        }

        BeginDrawing();
        ClearBackground(BLACK);
        BeginMode2D(camera);

        if (controls.showDebug && liveConfig.execution.useSpatialGrid) {
            simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode, 2, SKYBLUE, DARKBLUE);
        }

        for (const auto& bubble : sim.getBubbles()) {
            DrawCircleV({bubble.position.x, bubble.position.y}, bubble.radius, Color{120, 190, 255, 170});
            DrawCircleLines(static_cast<int>(bubble.position.x), static_cast<int>(bubble.position.y), bubble.radius, BLUE);
        }

        if (selectedBubble && *selectedBubble < sim.getBubbles().size()) {
            const Bubble& bubble = sim.getBubbles()[*selectedBubble];
            simfw::ui::drawSelectionRing(bubble.position, bubble.radius + SelectionRingPadding, YELLOW);
        }

        EndMode2D();

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 22};
            cursor.draw("bubbles", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");
            cursor.draw(TextFormat("Bursting: %s", liveConfig.enableBurst ? "ON" : "OFF"));
            simfw::ui::drawStats(cursor, sim.getStats());
            simfw::ui::drawSimulationBackendStatus(cursor, liveConfig, gridDebugMode);
            cursor.gap(6);
            simfw::ui::drawTunables(cursor, liveConfig, controls.selectedParameter);

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                simfw::ui::TextCursor controlsCursor = simfw::ui::makeRightSideControlCursor(360, 10, 20);
                simfw::ui::drawControlHints(controlsCursor, {
                    "Left mouse: spawn bubbles",
                    "Right mouse: select bubble",
                    "C: clear",
                    "Wheel: zoom",
                    "Middle mouse: pan",
                    "Backspace: reset camera",
                    "Space: pause",
                    "N: step",
                    "R: reset",
                    "B: toggle burst",
                    "Tab: select tunable",
                    "Left/Right: adjust",
                    "Shift: fast adjust",
                    "G: toggle grid backend",
                    "H: grid debug mode",
                    "P: parallel update"
                });
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
