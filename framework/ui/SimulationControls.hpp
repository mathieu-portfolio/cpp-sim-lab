#pragma once

#include "RaylibDebugUi.hpp"

#include <cstddef>

#include <raylib.h>

namespace simfw::ui {

struct SimulationControls {
    bool paused = false;
    bool step = false;
    bool showDebug = false;

    UiMode uiMode = UiMode::Compact;
    std::size_t selectedParameter = 0;
};

template <typename Simulation>
void handleCommonSimulationControls(
    SimulationControls& controls,
    Simulation& sim,
    std::size_t parameterCount
) {
    if (IsKeyPressed(KEY_SPACE)) {
        controls.paused = !controls.paused;
    }

    if (IsKeyPressed(KEY_R)) {
        sim.reset();
    }

    if (IsKeyPressed(KEY_D)) {
        controls.showDebug = !controls.showDebug;
    }

    if (IsKeyPressed(KEY_F1)) {
        controls.uiMode = nextUiMode(controls.uiMode);
    }

    if (parameterCount > 0) {
        controls.selectedParameter %= parameterCount;

        if (IsKeyPressed(KEY_TAB)) {
            controls.selectedParameter =
                (controls.selectedParameter + 1) % parameterCount;
        }
    } else {
        controls.selectedParameter = 0;
    }

    if (controls.paused && IsKeyPressed(KEY_N)) {
        controls.step = true;
    }
}

inline bool shouldAdvanceSimulation(const SimulationControls& controls) {
    return !controls.paused || controls.step;
}

inline void finishSimulationStep(SimulationControls& controls) {
    controls.step = false;
}

} // namespace simfw::ui
