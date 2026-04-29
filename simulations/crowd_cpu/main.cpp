#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <raylib.h>
#include <ui/EntitySelection.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <optional>

using namespace crowd_cpu;

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;
constexpr float AgentDrawRadius = 3.0f;
constexpr float SelectionPickRadius = 14.0f;
constexpr float SelectionRingRadius = 10.0f;

bool isSelectionModifierDown() {
    return IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
}

Vec2 agentPosition(const Agent& agent) {
    return agent.position;
}
} // namespace

int main() {
    InitWindow(WindowWidth, WindowHeight, "crowd_cpu");
    SetTargetFPS(60);

    Simulation sim;

    simfw::ui::SimulationControls controls;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;
    std::optional<std::size_t> selectedAgent;

    Camera2D camera = simfw::ui::makeCenteredCamera(
        static_cast<float>(WindowWidth),
        static_cast<float>(WindowHeight)
    );

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        auto& config = sim.getConfig();

        constexpr std::size_t paramCount =
            std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;

        simfw::ui::handleCommonSimulationControls(
            controls,
            sim,
            paramCount
        );

        simfw::ui::handleSimulationBackendControls(config, gridDebugMode);

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            const Vec2 mouseWorld = simfw::ui::screenToWorld(GetMousePosition(), camera);

            if (isSelectionModifierDown()) {
                selectedAgent = simfw::ui::findClosestEntity(
                    sim.getEntities(),
                    mouseWorld,
                    SelectionPickRadius / camera.zoom,
                    agentPosition
                );
            } else {
                sim.setGoal(mouseWorld);
            }
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            sim.addObstacle(simfw::ui::screenToWorld(GetMousePosition(), camera));
        }

        if (IsKeyPressed(KEY_C)) {
            sim.clearObstacles();
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

        if (selectedAgent && *selectedAgent >= sim.getEntities().size()) {
            selectedAgent.reset();
        }

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode2D(camera);

        if (controls.showDebug && config.execution.useSpatialGrid) {
            simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode);
        }

        for (const auto& agent : sim.getEntities()) {
            DrawCircleV(simfw::ui::toRaylib(agent.position), AgentDrawRadius, WHITE);
        }

        for (const auto& obstacle : sim.getObstacles()) {
            DrawCircleV(simfw::ui::toRaylib(obstacle.position), obstacle.radius, DARKGRAY);
        }

        if (controls.showDebug) {
            const auto& flow = sim.getFlowField();
            const std::size_t gridWidth = static_cast<std::size_t>(config.width / config.gridCellSize) + 1;
            const std::size_t gridHeight = static_cast<std::size_t>(config.height / config.gridCellSize) + 1;

            for (std::size_t y = 0; y < gridHeight; ++y) {
                for (std::size_t x = 0; x < gridWidth; ++x) {
                    const Vec2 direction = flow[y * gridWidth + x];
                    const Vec2 cellCenter{
                        (static_cast<float>(x) + 0.5f) * config.gridCellSize,
                        (static_cast<float>(y) + 0.5f) * config.gridCellSize
                    };

                    DrawLineV(
                        simfw::ui::toRaylib(cellCenter),
                        simfw::ui::toRaylib(cellCenter + direction * 8.0f),
                        SKYBLUE
                    );
                }
            }
        }

        DrawCircleLines(
            static_cast<int>(sim.getGoal().x),
            static_cast<int>(sim.getGoal().y),
            config.goalRadius,
            GREEN
        );

        if (selectedAgent) {
            simfw::ui::drawSelectionRing(
                sim.getEntities()[*selectedAgent].position,
                SelectionRingRadius,
                YELLOW
            );
        }

        EndMode2D();

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 20};

            cursor.draw("crowd_cpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");

            simfw::ui::drawStats(cursor, sim.getStats());
            simfw::ui::drawSimulationBackendStatus(cursor, config, gridDebugMode);

            cursor.gap(6);
            simfw::ui::drawTunables(cursor, config, controls.selectedParameter);

            cursor.gap(8);
            cursor.draw(TextFormat("Zoom: %.2fx", camera.zoom), 18, GRAY);

            if (selectedAgent) {
                const Agent& agent = sim.getEntities()[*selectedAgent];
                cursor.draw(TextFormat("Selected agent: %d", static_cast<int>(*selectedAgent)), 16, YELLOW);
                cursor.draw(TextFormat("pos: %.1f, %.1f", agent.position.x, agent.position.y));
                cursor.draw(TextFormat("vel: %.1f, %.1f", agent.velocity.x, agent.velocity.y));
            }

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                simfw::ui::TextCursor controlsCursor =
                    simfw::ui::makeRightSideControlCursor(350, 10, 20);
                simfw::ui::drawControlHints(controlsCursor);
            }
        }

        EndDrawing();
    }

    CloseWindow();
}
