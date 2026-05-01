#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <raylib.h>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include <ui/UiHelpers.hpp>

#include <tuple>

using namespace traffic_flow_cpu;

namespace {
constexpr int WindowWidth = 1100;
constexpr int WindowHeight = 700;
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "traffic_flow_cpu");
    SetTargetFPS(60);

    Simulation sim;
    simfw::ui::SimulationControls controls;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& config = sim.getConfig();
        constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);
        simfw::ui::handleTunableAdjustment(config, controls, dt);

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

        simfw::ui::TextCursor cursor{30, 24, 20};
        cursor.draw("traffic_flow_cpu", 20, RAYWHITE);
        cursor.draw(controls.paused ? "Paused" : "Running");
        cursor.draw(TextFormat("Vehicles: %d", static_cast<int>(vehicles.size())));
        simfw::ui::drawStats(cursor, stats);
        cursor.gap(6);
        simfw::ui::drawTunables(cursor, config, controls.selectedParameter);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
