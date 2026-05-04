#include "Simulation.hpp"

#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include <ui/UiHelpers.hpp>
#include "SimulationUiTraits.hpp"

#include <raylib.h>

#include <algorithm>
#include <cmath>

namespace {
using heat_grid_cpu::BoundaryMode;

Color heatColor(float t) {
    const float n = std::clamp((t + 1.0f) * 0.5f, 0.0f, 1.0f);
    return Color{
        static_cast<unsigned char>(255.0f * n),
        static_cast<unsigned char>(255.0f * (1.0f - std::abs((n - 0.5f) * 2.0f))),
        static_cast<unsigned char>(255.0f * (1.0f - n)),
        255
    };
}

const char* boundaryModeName(BoundaryMode mode) {
    switch (mode) {
        case BoundaryMode::Clamp: return "clamp";
        case BoundaryMode::Wrap: return "wrap";
        case BoundaryMode::Insulated: return "insulated";
        default: return "unknown";
    }
}
} // namespace

int main() {
    heat_grid_cpu::SimulationConfig config;
    config.gridWidth = 160;
    config.gridHeight = 120;
    config.cellSize = 6.0f;
    config.width = config.gridWidth * config.cellSize;
    config.height = config.gridHeight * config.cellSize;

    InitWindow(static_cast<int>(config.width), static_cast<int>(config.height), "heat_grid_cpu");
    SetTargetFPS(60);

    heat_grid_cpu::Simulation sim{config};
    sim.reset();
    simfw::ui::SimulationControls controls;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& simConfig = sim.getConfig();

        constexpr std::size_t paramCount =
            std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<heat_grid_cpu::SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);

        if (IsKeyPressed(KEY_B)) {
            if (simConfig.boundaryMode == BoundaryMode::Clamp) simConfig.boundaryMode = BoundaryMode::Wrap;
            else if (simConfig.boundaryMode == BoundaryMode::Wrap) simConfig.boundaryMode = BoundaryMode::Insulated;
            else simConfig.boundaryMode = BoundaryMode::Clamp;
        }

        const bool fastAdjust = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);
        if (IsKeyDown(KEY_LEFT)) simfw::ui::adjustTunables(simConfig, controls.selectedParameter, -1.0f, dt, fastAdjust);
        if (IsKeyDown(KEY_RIGHT)) simfw::ui::adjustTunables(simConfig, controls.selectedParameter, 1.0f, dt, fastAdjust);

        if (simfw::ui::shouldAdvanceSimulation(controls)) {
            sim.update(dt);
            simfw::ui::finishSimulationStep(controls);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        const auto& temperature = sim.getTemperature();
        for (std::size_t y = 0; y < simConfig.gridHeight; ++y) {
            for (std::size_t x = 0; x < simConfig.gridWidth; ++x) {
                const float t = temperature[y * simConfig.gridWidth + x];
                DrawRectangle(
                    static_cast<int>(x * simConfig.cellSize),
                    static_cast<int>(y * simConfig.cellSize),
                    static_cast<int>(simConfig.cellSize + 1.0f),
                    static_cast<int>(simConfig.cellSize + 1.0f),
                    heatColor(t)
                );
            }
        }

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 22};
            cursor.draw("heat_grid_cpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");
            simfw::ui::drawStats(cursor, sim.getStats());
            cursor.gap(6);
            simfw::ui::drawTunables(cursor, simConfig, controls.selectedParameter);
            cursor.gap(8);
            cursor.draw(TextFormat("Boundary mode: %s (B to cycle)", boundaryModeName(simConfig.boundaryMode)), 16, LIGHTGRAY);

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                auto controlsCursor = simfw::ui::makeRightSideControlCursor(320, 10, 20);
                simfw::ui::drawControlHints(
                    controlsCursor,
                    {
                        "B: cycle boundary mode",
                        "Space: pause",
                        "N: step",
                        "R: reset",
                        "F1: UI mode",
                        "Tab: select tunable",
                        "Left/Right: adjust",
                        "Shift: fast adjust"
                    }
                );
            }
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
