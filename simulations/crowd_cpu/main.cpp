#include "Simulation.hpp"
#include "SimulationUiTraits.hpp"

#include <raylib.h>
#include <ui/EntitySelection.hpp>
#include <ui/RaylibCamera.hpp>
#include <ui/RaylibDebugUi.hpp>
#include <ui/SimulationBackendControls.hpp>
#include <ui/SimulationControlHints.hpp>
#include <ui/SimulationControls.hpp>
#include <ui/SimulationUiRenderer.hpp>

#include <optional>

using namespace crowd_cpu;

int main(){
    InitWindow(800,800,"crowd_cpu"); SetTargetFPS(60); Simulation sim;
    simfw::ui::SimulationControls controls; simfw::ui::GridDebugMode gridMode=simfw::ui::GridDebugMode::None; std::optional<std::size_t> selected;
    Camera2D camera = simfw::ui::makeCenteredCamera(800.0f,800.0f);
    while(!WindowShouldClose()){
        float dt=GetFrameTime(); auto& config=sim.getConfig();
        constexpr std::size_t paramCount = std::tuple_size_v<decltype(simfw::ui::ConfigUiTraits<SimulationConfig>::fields)>;
        simfw::ui::handleCommonSimulationControls(controls, sim, paramCount); simfw::ui::handleSimulationBackendControls(config, gridMode);
        if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){ Vec2 m=simfw::ui::screenToWorld(GetMousePosition(),camera); if(IsKeyDown(KEY_LEFT_CONTROL)||IsKeyDown(KEY_RIGHT_CONTROL)){ selected = simfw::ui::findClosestEntity(sim.getEntities(),m,14.0f/camera.zoom,[](const Agent&a){return a.position;}); } else sim.setGoal(m);}        
        if(IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) sim.addObstacle(simfw::ui::screenToWorld(GetMousePosition(),camera));
        if(IsKeyPressed(KEY_C)) sim.clearObstacles();
        if(simfw::ui::shouldAdvanceSimulation(controls)){ sim.update(dt); simfw::ui::finishSimulationStep(controls);}        
        BeginDrawing(); ClearBackground(BLACK); BeginMode2D(camera);
        if(config.execution.useSpatialGrid) simfw::ui::drawSpatialGridDebug(sim.getGrid(), gridMode);
        for(const auto& a: sim.getEntities()) { DrawCircleV(simfw::ui::toRaylib(a.position),3.0f,WHITE); }
        for(const auto& o: sim.getObstacles()) DrawCircleV(simfw::ui::toRaylib(o.position),o.radius,DARKGRAY);
        const auto& flow=sim.getFlowField(); std::size_t w=static_cast<std::size_t>(config.width/config.gridCellSize)+1; std::size_t h=static_cast<std::size_t>(config.height/config.gridCellSize)+1;
        if(controls.showDebug){ for(std::size_t y=0;y<h;++y)for(std::size_t x=0;x<w;++x){Vec2 d=flow[y*w+x]; Vec2 c{(x+0.5f)*config.gridCellSize,(y+0.5f)*config.gridCellSize}; DrawLineV(simfw::ui::toRaylib(c),simfw::ui::toRaylib(c+d*8.0f),SKYBLUE);} }
        DrawCircleLines(static_cast<int>(sim.getGoal().x), static_cast<int>(sim.getGoal().y), config.goalRadius, GREEN);
        if(selected && *selected < sim.getEntities().size()) simfw::ui::drawSelectionRing(sim.getEntities()[*selected].position,10.0f,YELLOW);
        EndMode2D(); EndDrawing();
    }
    CloseWindow();
}
