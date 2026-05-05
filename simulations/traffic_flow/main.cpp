#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <raylib.h>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationFrameHelpers.hpp>
#include <ui/SimulationUiRenderer.hpp>
#include <ui/UiHelpers.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/SpatialGridDebugDraw.hpp>

#include <algorithm>
#include <optional>
#include <tuple>
#include <ui/EntitySelection.hpp>

using namespace traffic_flow;

namespace {
constexpr int WindowWidth = 1100;
constexpr int WindowHeight = 700;
constexpr float SubsegmentInterval = 36.0f;
constexpr float ClickSnapRadius = 18.0f;

struct RoadPick {
    Vec2 point{};
    bool edge = false;
};

std::optional<RoadPick> pickRoadSubsegment(const RoadNetwork& network, Vec2 mouse) {
    std::optional<RoadPick> best;
    float bestDistSq = ClickSnapRadius * ClickSnapRadius;

    for (const auto& road : network.roads) {
        if (road.controlPoints.size() < 2) continue;
        const Vec2 start = road.controlPoints.front();
        const Vec2 end = road.controlPoints.back();
        const float length = (end - start).length();
        const int subdivisions = std::max(1, static_cast<int>(std::round(length / SubsegmentInterval)));

        for (int i = 0; i <= subdivisions; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(subdivisions);
            const Vec2 p = start + (end - start) * t;
            const float distSq = (mouse - p).lengthSquared();
            if (distSq <= bestDistSq) {
                bestDistSq = distSq;
                best = RoadPick{p, i == 0 || i == subdivisions};
            }
        }
    }
    return best;
}
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "traffic_flow");
    SetTargetFPS(60);

    Simulation sim;
    simfw::ui::SimulationControls controls;
    simfw::ui::GridDebugMode gridDebugMode = simfw::ui::GridDebugMode::OccupiedCells;
    Camera2D camera = simfw::ui::makeCenteredCamera(static_cast<float>(WindowWidth), static_cast<float>(WindowHeight));
    std::optional<Vec2> pendingRoadStart;

    while (!WindowShouldClose()) {
        const float dt = GetFrameTime();
        auto& config = sim.getConfig();
        constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount);
        simfw::ui::handleTunableAdjustment(config, controls, dt);

        simfw::ui::zoomCameraAtScreenPoint(camera, GetMousePosition(), GetMouseWheelMove(), 0.1f, 0.5f, 8.0f,
            static_cast<float>(WindowWidth), static_cast<float>(WindowHeight));

        if (IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
            simfw::ui::panCameraByScreenDelta(camera, GetMouseDelta(), static_cast<float>(WindowWidth), static_cast<float>(WindowHeight));
        }
        if (IsKeyPressed(KEY_BACKSPACE)) {
            simfw::ui::resetCameraToBounds(camera, static_cast<float>(WindowWidth), static_cast<float>(WindowHeight));
        }

        simfw::ui::handleSimulationBackendControls(config, gridDebugMode);
        if (IsKeyPressed(KEY_T)) sim.generateTraffic();

        const Vector2 mouseScreen = GetMousePosition();
        const Vector2 mouseWorldRaylib = GetScreenToWorld2D(mouseScreen, camera);
        const Vec2 mouseWorld{mouseWorldRaylib.x, mouseWorldRaylib.y};

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            if (!pendingRoadStart.has_value()) {
                const std::optional<RoadPick> pick = pickRoadSubsegment(sim.getRoadNetwork(), mouseWorld);
                pendingRoadStart = pick.has_value() ? pick->point : mouseWorld;
            } else {
                if ((mouseWorld - *pendingRoadStart).lengthSquared() > 64.0f) {
                    RoadSegment road;
                    road.controlPoints = {*pendingRoadStart, mouseWorld};
                    road.lanes = {{1, -config.laneWidth * 0.5f}, {-1, config.laneWidth * 0.5f}};
                    sim.getRoadNetwork().roads.push_back(std::move(road));
                    sim.notifyRoadsEdited();
                }
                pendingRoadStart.reset();
            }
        }

        if (simfw::ui::shouldAdvanceSimulation(controls)) {
            sim.update(dt);
            simfw::ui::finishSimulationStep(controls);
        }

        BeginDrawing();
        ClearBackground(BLACK);

        const auto& vehicles = sim.getVehicles();
        const auto& stats = sim.getStats();

        BeginMode2D(camera);
        for (std::size_t roadId = 0; roadId < sim.getRoadNetwork().roads.size(); ++roadId) {
            const auto& road = sim.getRoadNetwork().roads[roadId];
            if (road.controlPoints.size() < 2) continue;
            const Vec2 a = road.controlPoints.front();
            const Vec2 b = road.controlPoints.back();
            DrawLineEx({a.x, a.y}, {b.x, b.y}, config.laneWidth * 2.0f, DARKGRAY);
            DrawLineEx({a.x, a.y}, {b.x, b.y}, 2.0f, YELLOW);

            const int subdivisions = std::max(1, static_cast<int>(std::round((b - a).length() / SubsegmentInterval)));
            for (int i = 0; i <= subdivisions; ++i) {
                const float t = static_cast<float>(i) / static_cast<float>(subdivisions);
                const Vec2 p = a + (b - a) * t;
                DrawCircleV({p.x, p.y}, (i == 0 || i == subdivisions) ? 5.0f : 3.0f, (i == 0 || i == subdivisions) ? ORANGE : GRAY);
            }
        }

        if (pendingRoadStart.has_value()) {
            DrawLineEx({pendingRoadStart->x, pendingRoadStart->y}, {mouseWorld.x, mouseWorld.y}, 2.0f, SKYBLUE);
            DrawCircleV({pendingRoadStart->x, pendingRoadStart->y}, 6.0f, SKYBLUE);
        }

        for (const auto& crossroad : sim.getRoadNetwork().crossroads) {
            DrawCircleV({crossroad.position.x, crossroad.position.y}, 8.0f, RED);
        }

        for (const Vehicle& v : vehicles) {
            const Vec2 p = sim.sampleLanePosition(v.roadId, v.laneId, v.s);
            const Color color = v.speed < 3.0f ? ORANGE : SKYBLUE;
            DrawRectangleV({p.x - 6.0f, p.y - 5.0f}, {12.0f, 10.0f}, color);
        }
        EndMode2D();

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
