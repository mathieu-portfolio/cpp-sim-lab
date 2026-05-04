#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <raylib.h>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationFrameHelpers.hpp>
#include <ui/SimulationUiRenderer.hpp>

using namespace predator_prey_cpu;

int main() {
  InitWindow(800, 800, "predator_prey_cpu");
  SetTargetFPS(60);
  Simulation sim;
  simfw::ui::SimulationControls controls;
  simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;

  while (!WindowShouldClose()) {
    const float dt = GetFrameTime();
    auto &config = sim.getConfig();
    constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
    simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);
    simfw::ui::handleSimulationBackendControls(config, gridDebugMode);
    simfw::ui::handleTunableAdjustment(config, controls, dt);
    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON)) sim.spawn({(float)GetMouseX(), (float)GetMouseY()});
    if (simfw::ui::shouldAdvanceSimulation(controls)) { sim.update(dt); simfw::ui::finishSimulationStep(controls); }

    BeginDrawing();
    ClearBackground(BLACK);
    for (const auto &b : sim.getBoids()) {
      const Color c = b.type == AgentType::Prey ? SKYBLUE : ORANGE;
      DrawCircleV({b.position.x, b.position.y}, b.type == AgentType::Prey ? 3.0f : 4.0f, c);
    }
    simfw::ui::TextCursor cursor{10, 10, 20};
    cursor.draw("predator_prey_cpu", 20, RAYWHITE);
    simfw::ui::drawStats(cursor, sim.getStats());
    simfw::ui::drawTunables(cursor, config, controls.selectedParameter);
    EndDrawing();
  }

  CloseWindow();
  return 0;
}
