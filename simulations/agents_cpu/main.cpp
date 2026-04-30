#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <ui/EntitySelection.hpp>
#include <ui/ObstacleMapPng.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <raylib.h>

#include <algorithm>
#include <optional>
#include <vector>

using namespace agents_cpu;

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;
constexpr float AgentDrawRadius = 3.0f;
constexpr float SelectionPickRadius = 14.0f;
constexpr float SelectionRingRadius = 10.0f;
constexpr float ObstacleBrushRadius = 16.0f;

void drawAgent(const Agent& agent) {
    DrawCircleV(simfw::ui::toRaylib(agent.position), AgentDrawRadius, WHITE);

    Vec2 dir = agent.velocity.length() > 0.001f
        ? agent.velocity.normalized()
        : Vec2{1.0f, 0.0f};

    Vec2 end = agent.position + dir * 10.0f;

    DrawLineV(
        simfw::ui::toRaylib(agent.position),
        simfw::ui::toRaylib(end),
        GRAY
    );
}

void drawObstacle(const Obstacle& obstacle) {
    DrawCircle(
        static_cast<int>(obstacle.position.x),
        static_cast<int>(obstacle.position.y),
        obstacle.radius,
        DARKGRAY
    );

    DrawCircleLines(
        static_cast<int>(obstacle.position.x),
        static_cast<int>(obstacle.position.y),
        obstacle.radius,
        GRAY
    );
}

void drawDebugObstacle(const Obstacle& obstacle, const SimulationConfig& config) {
    DrawCircleLines(
        static_cast<int>(obstacle.position.x),
        static_cast<int>(obstacle.position.y),
        obstacle.radius + config.obstacleAvoidanceRadius,
        DARKGRAY
    );
}

void drawDebugAgent(const Agent& agent, const SimulationConfig& config) {
    DrawCircleLines(
        static_cast<int>(agent.position.x),
        static_cast<int>(agent.position.y),
        config.separationRadius,
        DARKGRAY
    );

    DrawLineV(
        simfw::ui::toRaylib(agent.position),
        simfw::ui::toRaylib(agent.target),
        DARKGREEN
    );
}

void drawTarget(Vec2 target, const SimulationConfig& config) {
    DrawCircleLines(
        static_cast<int>(target.x),
        static_cast<int>(target.y),
        config.targetRadius,
        GREEN
    );

    DrawLine(
        static_cast<int>(target.x - 8.0f),
        static_cast<int>(target.y),
        static_cast<int>(target.x + 8.0f),
        static_cast<int>(target.y),
        GREEN
    );

    DrawLine(
        static_cast<int>(target.x),
        static_cast<int>(target.y - 8.0f),
        static_cast<int>(target.x),
        static_cast<int>(target.y + 8.0f),
        GREEN
    );
}

Vec2 agentPosition(const Agent& agent) {
    return agent.position;
}

bool isSelectionModifierDown() {
    return IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
}
} // namespace

int main() {
    SimulationConfig config;
    config.width = static_cast<float>(WindowWidth);
    config.height = static_cast<float>(WindowHeight);

    InitWindow(WindowWidth, WindowHeight, "agents_cpu");
    SetTargetFPS(60);

    Simulation sim{config};

    simfw::ui::SimulationControls controls;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;
    std::optional<std::size_t> selectedAgent;
    std::vector<uint8_t> obstaclePaintMask(
        static_cast<std::size_t>(WindowWidth * WindowHeight),
        0u
    );
    bool obstacleMaskDirty = false;

    Camera2D camera = simfw::ui::makeCenteredCamera(
        config.width,
        config.height
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
                    sim.getAgents(),
                    mouseWorld,
                    SelectionPickRadius / camera.zoom,
                    agentPosition
                );
            } else if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                sim.spawn(mouseWorld);
            } else {
                sim.setTarget(mouseWorld);
            }
        }

        if (IsMouseButtonDown(MOUSE_RIGHT_BUTTON)) {
            const Vec2 mouseWorld = simfw::ui::screenToWorld(GetMousePosition(), camera);
            if (simfw::ui::paintObstacleMaskCircle(
                    obstaclePaintMask,
                    WindowWidth,
                    WindowHeight,
                    mouseWorld,
                    ObstacleBrushRadius / camera.zoom
                )) {
                obstacleMaskDirty = true;
            }
        }

        if (IsKeyPressed(KEY_C)) {
            sim.clearObstacles();
            std::fill(obstaclePaintMask.begin(), obstaclePaintMask.end(), 0u);
            obstacleMaskDirty = false;
        }

        if (obstacleMaskDirty) {
            sim.clearObstacles();
            const auto circles = simfw::ui::buildObstacleCirclesFromMask(
                obstaclePaintMask,
                WindowWidth,
                WindowHeight
            );
            for (const auto& [position, radius] : circles) {
                (void)radius;
                sim.addObstacle(position);
            }
            obstacleMaskDirty = false;
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

        if (selectedAgent && *selectedAgent >= sim.getAgents().size()) {
            selectedAgent.reset();
        }

        BeginDrawing();
        ClearBackground(BLACK);

        BeginMode2D(camera);

        if (config.execution.useSpatialGrid) {
            simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode);
        }

        drawTarget(sim.getTarget(), config);

        for (const Obstacle& obstacle : sim.getObstacles()) {
            drawObstacle(obstacle);

            if (controls.showDebug) {
                drawDebugObstacle(obstacle, config);
            }
        }

        for (const Agent& agent : sim.getAgents()) {
            if (controls.showDebug) {
                drawDebugAgent(agent, config);
            }

            drawAgent(agent);
        }

        if (selectedAgent) {
            simfw::ui::drawSelectionRing(
                sim.getAgents()[*selectedAgent].position,
                SelectionRingRadius,
                YELLOW
            );
        }

        EndMode2D();

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 20};

            cursor.draw("agents_cpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");

            simfw::ui::drawStats(cursor, sim.getStats());

            simfw::ui::drawSimulationBackendStatus(cursor, config, gridDebugMode);

            cursor.gap(6);

            simfw::ui::drawTunables(
                cursor,
                config,
                controls.selectedParameter
            );

            cursor.gap(8);
            cursor.draw(
                TextFormat("Zoom: %.2fx", camera.zoom),
                18,
                GRAY
            );

            if (selectedAgent) {
                const Agent& agent = sim.getAgents()[*selectedAgent];
                cursor.draw(TextFormat("Selected agent: %d", static_cast<int>(*selectedAgent)), 16, YELLOW);
                cursor.draw(TextFormat("intent: %s", intentName(agent.intent)));
                cursor.draw(TextFormat("pos: %.1f, %.1f", agent.position.x, agent.position.y));
                cursor.draw(TextFormat("vel: %.1f, %.1f", agent.velocity.x, agent.velocity.y));
            }

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                simfw::ui::TextCursor controlsCursor =
                    simfw::ui::makeRightSideControlCursor(340, 10, 20);

                simfw::ui::drawControlHints(
                    controlsCursor,
                    {
                        "Left mouse: set target",
                        "Ctrl + Left mouse: select agent",
                        "Right mouse: add obstacle",
                        "C: clear obstacles",
                        "Wheel: zoom",
                        "Middle mouse: pan",
                        "Backspace: reset camera",
                        "Space: pause",
                        "N: step",
                        "R: reset",
                        "D: debug",
                        "F1: UI mode",
                        "Tab: select tunable",
                        "Left/Right: adjust",
                        "Shift: fast adjust",
                        "G: toggle grid backend",
                        "H: grid debug mode",
                        "P: parallel update"
                    }
                );
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
