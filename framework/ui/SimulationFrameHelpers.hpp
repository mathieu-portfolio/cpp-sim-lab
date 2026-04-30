#pragma once

#include "RaylibCamera.hpp"
#include "SimulationControls.hpp"
#include "SimulationUiRenderer.hpp"

#include <raylib.h>

namespace simfw::ui {

template <typename Config>
inline void handleTunableAdjustment(Config &config,
                                    const SimulationControls &controls,
                                    float dt) {
  const bool fastAdjust =
      IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

  if (IsKeyDown(KEY_LEFT)) {
    adjustTunables(config, controls.selectedParameter, -1.0f, dt, fastAdjust);
  }

  if (IsKeyDown(KEY_RIGHT)) {
    adjustTunables(config, controls.selectedParameter, 1.0f, dt, fastAdjust);
  }
}

template <typename Config>
inline void handleCameraInput(Camera2D &camera, const Config &config) {
  zoomCameraAtScreenPoint(camera, GetMousePosition(), GetMouseWheelMove(), 0.1f,
                          1.0f, 8.0f, config.width, config.height);

  if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
    panCameraByScreenDelta(camera, GetMouseDelta(), config.width,
                           config.height);
  }

  if (IsKeyPressed(KEY_BACKSPACE)) {
    resetCameraToBounds(camera, config.width, config.height);
  }
}

} // namespace simfw::ui
