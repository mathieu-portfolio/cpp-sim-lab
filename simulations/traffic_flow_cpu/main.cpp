#include "Simulation.hpp"

#include <raylib.h>
#include <ui/UiHelpers.hpp>

using namespace traffic_flow_cpu;

namespace {
constexpr int WindowWidth = 1100;
constexpr int WindowHeight = 700;
}

int main() {
    InitWindow(WindowWidth, WindowHeight, "traffic_flow_cpu");
    SetTargetFPS(60);

    Simulation sim;

    while (!WindowShouldClose()) {
        sim.update(GetFrameTime());

        BeginDrawing();
        ClearBackground(BLACK);

        const auto& config = sim.getConfig();
        const auto& vehicles = sim.getVehicles();
        const auto& stats = sim.getStats();

        const float roadX = 80.0f;
        const float roadY = 150.0f;
        const float roadW = 920.0f;
        const float roadH = config.laneWidth * static_cast<float>(config.laneCount);

        DrawRectangleLinesEx({roadX, roadY, roadW, roadH}, 2.0f, DARKGRAY);
        for (int lane = 1; lane < config.laneCount; ++lane) {
            const float y = roadY + config.laneWidth * static_cast<float>(lane);
            DrawLineEx({roadX, y}, {roadX + roadW, y}, 1.0f, GRAY);
        }

        for (const Vehicle& v : vehicles) {
            const float x = roadX + (v.s / config.roadLength) * roadW;
            const float y = roadY + (static_cast<float>(v.lane) + 0.5f) * config.laneWidth;
            const Color color = v.speed < 3.0f ? ORANGE : SKYBLUE;
            DrawRectangleV({x - 6.0f, y - 5.0f}, {12.0f, 10.0f}, color);
        }

        DrawText(TextFormat("Vehicles: %d", static_cast<int>(vehicles.size())), 30, 24, 20, RAYWHITE);
        DrawText(TextFormat("Throughput (veh/s): %.2f", stats.throughputPerSecond), 30, 52, 20, RAYWHITE);
        DrawText(TextFormat("Average speed (m/s): %.2f", stats.averageSpeed), 30, 80, 20, RAYWHITE);
        DrawText(TextFormat("Average queue length: %.2f", stats.averageQueueLength), 30, 108, 20, RAYWHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
