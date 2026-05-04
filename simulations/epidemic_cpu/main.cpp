#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <ui/SimulationControls.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/UiHelpers.hpp>
#include <ui/SpatialGridDebugDraw.hpp>

#include <raylib.h>

using namespace epidemic_cpu;

int main() {
    constexpr int W = 800, H = 800;
    InitWindow(W, H, "epidemic_cpu");
    SetTargetFPS(60);

    SimulationConfig config;
    config.width = static_cast<float>(W);
    config.height = static_cast<float>(H);
    Simulation sim(config);
    simfw::ui::SimulationControls controls;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& cfg = sim.getConfig();
        constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);
        simfw::ui::handleSimulationBackendControls(cfg, gridDebugMode);

        if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) sim.spawn(Vec2{static_cast<float>(GetMouseX()), static_cast<float>(GetMouseY())});
        if (simfw::ui::shouldAdvanceSimulation(controls)) { sim.update(dt); simfw::ui::finishSimulationStep(controls); }

        BeginDrawing();
        ClearBackground(BLACK);
        if (cfg.execution.useSpatialGrid) simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode);
        for (const Agent& a : sim.getAgents()) {
            Color c = a.infectionState == InfectionState::Susceptible ? BLUE : (a.infectionState == InfectionState::Infected ? RED : GREEN);
            DrawCircleV(simfw::ui::toRaylib(a.position), 3.0f, c);
        }
        simfw::ui::TextCursor cursor{10,10,20};
        cursor.draw("epidemic_cpu", 20, RAYWHITE);
        simfw::ui::drawStats(cursor, sim.getStats());
        simfw::ui::drawTunables(cursor, cfg, controls.selectedParameter);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
