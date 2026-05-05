#pragma once

#include "Simulation.hpp"

namespace traffic_flow {

class RoadGeometryCacheBuilder {
public:
    static void rebuild(SimulationConfig& config, RoadSegment& road);
    static void rebuildAll(SimulationConfig& config, RoadNetwork& network);
};

class CrossroadDetector {
public:
    static void rebuild(const SimulationConfig& config, const Simulation& simulation, RoadNetwork& network);
    static void rebuildRoadConnections(RoadNetwork& network);
};

} // namespace traffic_flow
