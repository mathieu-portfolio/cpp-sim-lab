#include "Simulation.hpp"

#include <algorithm>
#include <array>
#include <raylib.h>

namespace {
constexpr int WindowWidth = 800;
constexpr int WindowHeight = 800;

enum class UiMode {
    None,
    Compact,
    Full
};

struct TunableParameter {
    const char* name;
    float* value;
    float minValue;
    float normalStep;
    float fastStep;
};

Vector2 toRaylib(Vec2 v) {
    return Vector2{v.x, v.y};
}

float clampMin(float value, float minValue) {
    return std::max(value, minValue);
}

UiMode nextUiMode(UiMode mode) {
    return static_cast<UiMode>((static_cast<int>(mode) + 1) % 3);
}

void drawBoid(const Boid& b) {
    DrawCircleV(toRaylib(b.position), 3.0f, WHITE);

    Vec2 dir = b.velocity.length() > 0.001f
        ? b.velocity.normalized()
        : Vec2{1.0f, 0.0f};

    Vec2 end = b.position + dir * 10.0f;

    DrawLineV(toRaylib(b.position), toRaylib(end), GRAY);
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

void drawCompactUi(
    bool paused,
    const Simulation& sim,
    const TunableParameter& selected
) {
    const SimulationStats stats = sim.getStats();

    int y = 10;
    const int lineHeight = 20;

    DrawText("boids_cpu", 10, y, 20, RAYWHITE);
    y += lineHeight + 6;

    DrawText(
        paused ? "Paused" : "Running",
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Boids: %d", static_cast<int>(stats.boidCount)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Neighbor checks: %d", static_cast<int>(stats.neighborChecks)),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat("Selected: %s = %.2f", selected.name, *selected.value),
        10,
        y,
        16,
        YELLOW
    );
    y += lineHeight;

    DrawText(
        "F1: UI mode",
        10,
        y,
        16,
        DARKGRAY
    );
}

void drawFullUi(const SimulationConfig& config, int startY) {
    int y = startY;
    const int lineHeight = 20;

    DrawText(
        "Space: pause | N: step | R: reset | D: debug",
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        "Tab: select | Left/Right: adjust | Shift: fast",
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat(
            "Align %.2f | Cohesion %.2f | Separation %.2f",
            config.alignmentWeight,
            config.cohesionWeight,
            config.separationWeight
        ),
        10,
        y,
        16,
        LIGHTGRAY
    );
    y += lineHeight;

    DrawText(
        TextFormat(
            "Perception %.1f | Separation radius %.1f",
            config.perceptionRadius,
            config.separationRadius
        ),
        10,
        y,
        16,
        LIGHTGRAY
    );
}
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "boids_cpu");
    SetTargetFPS(60);

    Simulation sim;

    bool paused = false;
    bool showDebug = false;
    UiMode uiMode = UiMode::Compact;
    std::size_t selectedParameter = 0;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();

        auto& config = sim.getConfig();

        std::array<TunableParameter, 5> parameters{{
            {"alignmentWeight", &config.alignmentWeight, 0.0f, 0.5f, 2.0f},
            {"cohesionWeight", &config.cohesionWeight, 0.0f, 0.5f, 2.0f},
            {"separationWeight", &config.separationWeight, 0.0f, 0.5f, 2.0f},
            {"perceptionRadius", &config.perceptionRadius, 1.0f, 30.0f, 100.0f},
            {"separationRadius", &config.separationRadius, 1.0f, 30.0f, 100.0f},
        }};

        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_R)) sim.reset();
        if (IsKeyPressed(KEY_D)) showDebug = !showDebug;
        if (IsKeyPressed(KEY_F1)) uiMode = nextUiMode(uiMode);

        if (IsKeyPressed(KEY_TAB)) {
            selectedParameter = (selectedParameter + 1) % parameters.size();
        }

        const bool fastAdjust =
            IsKeyDown(KEY_LEFT_SHIFT) ||
            IsKeyDown(KEY_RIGHT_SHIFT);

        TunableParameter& selected = parameters[selectedParameter];
        const float step = (fastAdjust ? selected.fastStep : selected.normalStep) * dt;

        if (IsKeyDown(KEY_LEFT)) {
            *selected.value = clampMin(*selected.value - step, selected.minValue);
        }

        if (IsKeyDown(KEY_RIGHT)) {
            *selected.value = clampMin(*selected.value + step, selected.minValue);
        }

        const bool stepFrame = paused && IsKeyPressed(KEY_N);

        if (!paused || stepFrame) {
            sim.update(dt);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        for (const auto& b : sim.getBoids()) {
            if (showDebug) {
                drawDebugRadii(b, config);
            }
            drawBoid(b);
        }

        if (uiMode != UiMode::None) {
            drawCompactUi(paused, sim, selected);
        }

        if (uiMode == UiMode::Full) {
            drawFullUi(config, 140);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}