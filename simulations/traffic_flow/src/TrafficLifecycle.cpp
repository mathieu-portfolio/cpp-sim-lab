#include "TrafficLifecycle.hpp"

#include "TrafficPhysics.hpp"

#include <random/Random.hpp>

namespace traffic_flow {

void TrafficResetService::clear(std::vector<Vehicle>& vehicles, SimulationStats& stats, TrafficState& state) {
    vehicles.clear();
    stats = {};
    state = {};
}

void TrafficSpawner::generate(const Simulation& simulation, std::vector<Vehicle>& vehicles) {
    const auto& network = simulation.getRoadNetwork();
    const auto& config = simulation.getConfig();
    vehicles.clear();
    std::vector<std::size_t> spawnRoads;
    for (std::size_t roadId = 0; roadId < network.roads.size(); ++roadId) {
        const RoadSegment& road = network.roads[roadId];
        if (road.length > 1.0f && !road.lanes.empty()) spawnRoads.push_back(roadId);
    }
    if (spawnRoads.empty()) return;
    std::vector<std::size_t> perRoadTarget(spawnRoads.size(), 0);
    for (std::size_t i = 0; i < config.vehicleCount; ++i) ++perRoadTarget[i % spawnRoads.size()];
    for (std::size_t slot = 0; slot < spawnRoads.size(); ++slot) {
        const std::size_t roadId = spawnRoads[slot]; const RoadSegment& road = network.roads[roadId]; const std::size_t roadTarget = perRoadTarget[slot]; if (!roadTarget) continue;
        std::vector<std::size_t> perLaneTarget(road.lanes.size(), 0); for (std::size_t i = 0; i < roadTarget; ++i) ++perLaneTarget[i % road.lanes.size()];
        for (std::size_t laneIx = 0; laneIx < road.lanes.size(); ++laneIx) {
            const std::size_t targetCount = perLaneTarget[laneIx]; if (!targetCount) continue; const float spacing = std::max(config.spawnMinimumGap, config.minimumGap + 4.5f);
            const std::size_t maxByLength = static_cast<std::size_t>(std::max(1.0f, std::floor(road.length / spacing))); const std::size_t count = std::min(targetCount, maxByLength);
            for (std::size_t i = 0; i < count; ++i) {
                const float s = spacing * (0.5f + static_cast<float>(i));
                if (s >= road.length) break;
                Vehicle v; v.roadId=roadId; v.laneId=static_cast<int>(laneIx); v.s=s; v.speed=Random::range(config.spawnSpeedMin, config.spawnSpeedMax); vehicles.push_back(v);
            }
        }
    }
}

} // namespace traffic_flow
