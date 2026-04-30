#include "Simulation.hpp"

#include "SimulationUiTraits.hpp"
#include <ui/EntitySelection.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationFrameHelpers.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <raylib.h>

#include <optional>

using namespace boids_cpu;

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;
constexpr float BoidDrawRadius = 3.0f;
constexpr float SelectionPickRadius = 14.0f;
constexpr float SelectionRingRadius = 10.0f;

void drawBoid(const Boid &b) {
  DrawCircleV(simfw::ui::toRaylib(b.position), BoidDrawRadius, WHITE);

  Vec2 dir =
      b.velocity.length() > 0.001f ? b.velocity.normalized() : Vec2{1.0f, 0.0f};

  Vec2 end = b.position + dir * 10.0f;

  DrawLineV(simfw::ui::toRaylib(b.position), simfw::ui::toRaylib(end), GRAY);
}

void drawDebugRadii(const Boid &b, const SimulationConfig &config) {
  DrawCircleLines(static_cast<int>(b.position.x),
                  static_cast<int>(b.position.y), config.perceptionRadius,
                  DARKGRAY);

  DrawCircleLines(static_cast<int>(b.position.x),
                  static_cast<int>(b.position.y), config.separationRadius,
                  GRAY);
}

Vec2 boidPosition(const Boid &boid) { return boid.position; }

bool isSelectionModifierDown() {
  return IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
}
} // namespace

int main() {
  InitWindow(WindowWidth, WindowHeight, "boids_cpu");
  SetTargetFPS(60);

  Simulation sim;

  simfw::ui::SimulationControls controls;
  simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::None;
  std::optional<std::size_t> selectedBoid;

  Camera2D camera = simfw::ui::makeCenteredCamera(
      static_cast<float>(WindowWidth), static_cast<float>(WindowHeight));

  while (!WindowShouldClose()) {
    const float dt = GetFrameTime();

    auto &config = sim.getConfig();

    constexpr std::size_t paramCount = std::tuple_size_v<
        decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;

    simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);

    simfw::ui::handleSimulationBackendControls(config, gridDebugMode);

    simfw::ui::handleTunableAdjustment(config, controls, dt);

    if (IsMouseButtonDown(MOUSE_LEFT_BUTTON) && !isSelectionModifierDown()) {
      sim.spawn(simfw::ui::screenToWorld(GetMousePosition(), camera));
    }

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && isSelectionModifierDown()) {
      selectedBoid = simfw::ui::findClosestEntity(
          sim.getBoids(), simfw::ui::screenToWorld(GetMousePosition(), camera),
          SelectionPickRadius / camera.zoom, boidPosition);
    }

    if (simfw::ui::shouldAdvanceSimulation(controls)) {
      sim.update(dt);
      simfw::ui::finishSimulationStep(controls);
    }

    simfw::ui::handleCameraInput(camera, config);

    if (selectedBoid && *selectedBoid >= sim.getBoids().size()) {
      selectedBoid.reset();
    }

    BeginDrawing();
    ClearBackground(BLACK);

    BeginMode2D(camera);

    if (config.execution.useSpatialGrid) {
      simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridDebugMode);
    }

    for (const auto &b : sim.getBoids()) {
      if (controls.showDebug) {
        drawDebugRadii(b, config);
      }

      drawBoid(b);
    }

    if (selectedBoid) {
      simfw::ui::drawSelectionRing(sim.getBoids()[*selectedBoid].position,
                                   SelectionRingRadius, YELLOW);
    }

    EndMode2D();

    if (controls.uiMode != simfw::ui::UiMode::None) {
      simfw::ui::TextCursor cursor{10, 10, 20};

      cursor.draw("boids_cpu", 20, RAYWHITE);
      cursor.draw(controls.paused ? "Paused" : "Running");

      simfw::ui::drawStats(cursor, sim.getStats());

      simfw::ui::drawSimulationBackendStatus(cursor, config, gridDebugMode);

      cursor.gap(6);

      simfw::ui::drawTunables(cursor, config, controls.selectedParameter);

      cursor.gap(8);
      cursor.draw(TextFormat("Zoom: %.2fx", camera.zoom), 18, GRAY);

      if (selectedBoid) {
        const Boid &boid = sim.getBoids()[*selectedBoid];
        cursor.draw(
            TextFormat("Selected boid: %d", static_cast<int>(*selectedBoid)),
            16, YELLOW);
        cursor.draw(
            TextFormat("pos: %.1f, %.1f", boid.position.x, boid.position.y));
        cursor.draw(
            TextFormat("vel: %.1f, %.1f", boid.velocity.x, boid.velocity.y));
      }

      if (controls.uiMode == simfw::ui::UiMode::Full) {
        simfw::ui::TextCursor controlsCursor =
            simfw::ui::makeRightSideControlCursor(320, 10, 20);

        simfw::ui::drawControlHints(
            controlsCursor,
            {"Ctrl + Left mouse: select boid", "Wheel: zoom",
             "Middle mouse: pan", "Backspace: reset camera", "Space: pause",
             "N: step", "R: reset", "D: debug radii", "F1: UI mode",
             "Tab: select tunable", "Left/Right: adjust", "Shift: fast adjust",
             "G: toggle grid backend", "H: grid debug mode",
             "P: parallel update"});
      }
    }

    EndDrawing();
  }

  CloseWindow();
  return 0;
}
