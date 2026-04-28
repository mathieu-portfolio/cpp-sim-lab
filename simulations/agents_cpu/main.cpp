#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <raylib.h>

using namespace agents_cpu;

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;

void drawAgent(const Agent& agent) {
    DrawCircleV(simfw::ui::toRaylib(agent.position), 3.0f, WHITE);

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
            const Vector2 mouse = GetMousePosition();
            sim.setTarget(Vec2{mouse.x, mouse.y});
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            const Vector2 mouse = GetMousePosition();
            sim.addObstacle(Vec2{mouse.x, mouse.y});
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

        BeginDrawing();
        ClearBackground(BLACK);

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

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                simfw::ui::TextCursor controlsCursor =
                    simfw::ui::makeRightSideControlCursor(320, 10, 20);

                simfw::ui::drawControlHints(
                    controlsCursor,
                    {
                        "Left mouse: set target",
                        "Right mouse: add obstacle",
                        "C: clear obstacles",
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
