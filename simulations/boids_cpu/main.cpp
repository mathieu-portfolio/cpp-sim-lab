#include "Simulation.hpp"

#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include "SimulationUiTraits.hpp"

#include <raylib.h>

using namespace boids_cpu;

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;

void drawBoid(const Boid& b) {
    DrawCircleV(simfw::ui::toRaylib(b.position), 3.0f, WHITE);

    Vec2 dir = b.velocity.length() > 0.001f
        ? b.velocity.normalized()
        : Vec2{1.0f, 0.0f};

    Vec2 end = b.position + dir * 10.0f;

    DrawLineV(
        simfw::ui::toRaylib(b.position),
        simfw::ui::toRaylib(end),
        GRAY
    );
}

void drawDebugRadii(const Boid& b, const SimulationConfig& config) {
    DrawCircleLines(
        static_cast<int>(b.position.x),
        static_cast<int>(b.position.y),
        config.perceptionRadius,
        DARKGRAY
    );

    DrawCircleLines(
        static_cast<int>(b.position.x),
        static_cast<int>(b.position.y),
        config.separationRadius,
        GRAY
    );
}

} // namespace

int main() {
    InitWindow(WindowWidth, WindowHeight, "boids_cpu");
    SetTargetFPS(60);

    Simulation sim;

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

        for (const auto& b : sim.getBoids()) {
            if (controls.showDebug) {
                drawDebugRadii(b, config);
            }

            drawBoid(b);
        }

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 20};

            cursor.draw("boids_cpu", 20, RAYWHITE);
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
                cursor.draw("Space: pause | N: step | R: reset | D: debug radii | F1: UI mode");
                cursor.draw("Tab: select | Left/Right: adjust | Shift: fast");
                cursor.draw("G: toggle grid backend | H: grid debug mode");
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
