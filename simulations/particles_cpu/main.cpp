#include "Simulation.hpp"

#include "SimulationUiTraits.hpp"
#include <math/Vec2.hpp>
#include <ui/EntitySelection.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/SpatialGridDebugDraw.hpp>
#include <ui/UiHelpers.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationFrameHelpers.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <raylib.h>

#include <optional>

using namespace particles_cpu;

namespace {
constexpr float SelectionPickRadius = 14.0f;
constexpr float SelectionRingPadding = 5.0f;

Vec2 particlePosition(const Particle &particle) { return particle.position; }

bool isSelectionModifierDown() {
  return IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
}
} // namespace

int main() {
  SimulationConfig config;
  config.width = 800.0f;
  config.height = 800.0f;
  config.maxParticleCount = 1000;

  InitWindow(static_cast<int>(config.width), static_cast<int>(config.height),
             "particles_cpu");

  SetTargetFPS(60);

  Simulation sim{config};
  sim.reset();

  simfw::ui::SimulationControls controls;
  simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;
  std::optional<std::size_t> selectedParticle;

  Camera2D camera = simfw::ui::makeCenteredCamera(config.width, config.height);

  while (!WindowShouldClose()) {
    const float dt = GetFrameTime();

    auto &config = sim.getConfig();

    constexpr std::size_t paramCount = std::tuple_size_v<
        decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;

    simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);

    simfw::ui::handleSimulationBackendControls(config, gridDebugMode);

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && isSelectionModifierDown()) {
      selectedParticle = simfw::ui::findClosestEntity(
          sim.getParticles(),
          simfw::ui::screenToWorld(GetMousePosition(), camera),
          SelectionPickRadius / camera.zoom, particlePosition);
    } else if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) &&
               !isSelectionModifierDown()) {
      sim.spawn(simfw::ui::screenToWorld(GetMousePosition(), camera));
    }

    if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {
      sim.clear();
      selectedParticle.reset();
    }

    simfw::ui::handleTunableAdjustment(config, controls, dt);

    if (simfw::ui::shouldAdvanceSimulation(controls)) {
      sim.update(dt);
      simfw::ui::finishSimulationStep(controls);
    }

    simfw::ui::handleCameraInput(camera, config);

    if (selectedParticle && *selectedParticle >= sim.getParticles().size()) {
      selectedParticle.reset();
    }

    BeginDrawing();
    ClearBackground(BLACK);

    BeginMode2D(camera);

    if (controls.showDebug && config.execution.useSpatialGrid) {
      simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode, 2, YELLOW,
                                      ORANGE);
    }

    for (const auto &p : sim.getParticles()) {
      DrawCircle(static_cast<int>(p.position.x), static_cast<int>(p.position.y),
                 p.radius, WHITE);
    }

    if (selectedParticle) {
      const Particle &particle = sim.getParticles()[*selectedParticle];
      simfw::ui::drawSelectionRing(
          particle.position, particle.radius + SelectionRingPadding, YELLOW);
    }

    EndMode2D();

    if (controls.uiMode != simfw::ui::UiMode::None) {
      simfw::ui::TextCursor cursor{10, 10, 22};

      cursor.draw("particles_cpu", 20, RAYWHITE);
      cursor.draw(controls.paused ? "Paused" : "Running");

      simfw::ui::drawStats(cursor, sim.getStats());

      simfw::ui::drawSimulationBackendStatus(cursor, config, gridDebugMode);

      cursor.gap(6);

      simfw::ui::drawTunables(cursor, config, controls.selectedParameter);

      cursor.gap(8);
      cursor.draw(TextFormat("Zoom: %.2fx", camera.zoom), 18, GRAY);

      if (selectedParticle) {
        const Particle &particle = sim.getParticles()[*selectedParticle];
        cursor.draw(TextFormat("Selected particle: %d",
                               static_cast<int>(*selectedParticle)),
                    16, YELLOW);
        cursor.draw(TextFormat("pos: %.1f, %.1f", particle.position.x,
                               particle.position.y));
        cursor.draw(TextFormat("vel: %.1f, %.1f", particle.velocity.x,
                               particle.velocity.y));
      }

      if (controls.uiMode == simfw::ui::UiMode::Full) {
        simfw::ui::TextCursor controlsCursor =
            simfw::ui::makeRightSideControlCursor(330, 10, 20);

        simfw::ui::drawControlHints(
            controlsCursor,
            {"Left mouse: spawn", "Ctrl + Left mouse: select particle",
             "Right mouse: clear", "Wheel: zoom", "Middle mouse: pan",
             "Backspace: reset camera", "Space: pause", "N: step", "R: reset",
             "D: debug grid", "F1: UI mode", "Tab: select tunable",
             "Left/Right: adjust", "Shift: fast adjust",
             "G: toggle grid backend", "H: grid debug mode",
             "P: parallel update"});
      }
    }

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
