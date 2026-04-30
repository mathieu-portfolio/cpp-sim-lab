#include "Simulation.hpp"

#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include "SimulationUiTraits.hpp"

#include <raylib.h>

namespace {
using SandMaterial = sand_cpu::Material;

Color materialColor(SandMaterial material) {
    switch (material) {
        case SandMaterial::Sand:
            return Color{220, 190, 90, 255};
        case SandMaterial::Water:
            return Color{60, 130, 255, 255};
        case SandMaterial::Smoke:
            return Color{170, 170, 170, 220};
        case SandMaterial::Empty:
        default:
            return BLANK;
    }
}
} // namespace

int main() {
    sand_cpu::SimulationConfig config;
    config.gridWidth = 256;
    config.gridHeight = 256;
    config.cellSize = 3.0f;
    config.width = config.gridWidth * config.cellSize;
    config.height = config.gridHeight * config.cellSize;
    config.maxParticleCount = config.gridWidth * config.gridHeight;

    InitWindow(static_cast<int>(config.width), static_cast<int>(config.height), "sand_cpu");
    SetTargetFPS(60);

    sand_cpu::Simulation sim{config};
    sim.reset();

    simfw::ui::SimulationControls controls;
    SandMaterial brushMaterial = SandMaterial::Sand;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& simConfig = sim.getConfig();

        constexpr std::size_t paramCount =
            std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<sand_cpu::SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);

        if (IsKeyPressed(KEY_ONE)) brushMaterial = SandMaterial::Sand;
        if (IsKeyPressed(KEY_TWO)) brushMaterial = SandMaterial::Water;
        if (IsKeyPressed(KEY_THREE)) brushMaterial = SandMaterial::Smoke;

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
            const Vector2 mouse = GetMousePosition();
            const int cellX = static_cast<int>(mouse.x / simConfig.cellSize);
            const int cellY = static_cast<int>(mouse.y / simConfig.cellSize);
            sim.spawnDisc(cellX, cellY, brushMaterial);
        }

        if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
            sim.clear();
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

        const auto& cells = sim.getCells();
        for (std::size_t y = 0; y < simConfig.gridHeight; ++y) {
            for (std::size_t x = 0; x < simConfig.gridWidth; ++x) {
                const sand_cpu::Cell& cell = cells[y * simConfig.gridWidth + x];
                if (cell.material == SandMaterial::Empty) {
                    continue;
                }

                DrawRectangle(
                    static_cast<int>(x * simConfig.cellSize),
                    static_cast<int>(y * simConfig.cellSize),
                    static_cast<int>(simConfig.cellSize + 1.0f),
                    static_cast<int>(simConfig.cellSize + 1.0f),
                    materialColor(cell.material)
                );
            }
        }

        if (controls.uiMode != simfw::ui::UiMode::None) {
            simfw::ui::TextCursor cursor{10, 10, 22};
            cursor.draw("sand_cpu", 20, RAYWHITE);
            cursor.draw(controls.paused ? "Paused" : "Running");
            simfw::ui::drawStats(cursor, sim.getStats());
            cursor.gap(6);
            simfw::ui::drawTunables(cursor, simConfig, controls.selectedParameter);
            cursor.gap(8);
            cursor.draw("Brush: 1=sand, 2=water, 3=smoke", 16, LIGHTGRAY);

            if (controls.uiMode == simfw::ui::UiMode::Full) {
                auto controlsCursor = simfw::ui::makeRightSideControlCursor(320, 10, 20);
                simfw::ui::drawControlHints(
                    controlsCursor,
                    {
                        "Left mouse: paint",
                        "Right mouse: clear",
                        "1/2/3: material",
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
