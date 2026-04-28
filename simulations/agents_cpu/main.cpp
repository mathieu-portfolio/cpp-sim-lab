#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationControls.hpp>
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

        if (IsKeyPressed(KEY_G)) {
            config.useSpatialGrid = !config.useSpatialGrid;
        }

        if (IsKeyPressed(KEY_H)) {
            gridDebugMode = simfw::ui::nextGridDebugMode(gridDebugMode);
        }

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

        simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode);

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

            cursor.draw(
                config.useSpatialGrid ? "Backend: spatial grid" : "Backend: naive",
                16,
                config.useSpatialGrid ? GREEN : LIGHTGRAY
            );

            cursor.draw(
                TextFormat(
                    "Grid debug: %s",
                    simfw::ui::gridDebugModeName(gridDebugMode)
                )
            );

            cursor.gap(6);

            simfw::ui::drawTunables(
                cursor,
                config,
                controls.selectedParameter
            );

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                cursor.gap(10);
                cursor.draw("Left mouse: set target | Right mouse: add obstacle | C: clear obstacles");
                cursor.draw("Space: pause | N: step | R: reset | D: debug | F1: UI mode");
                cursor.draw("Tab: select | Left/Right: adjust | Shift: fast");
                cursor.draw("G: toggle grid backend | H: grid debug mode");
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
