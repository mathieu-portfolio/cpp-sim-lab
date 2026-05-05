#pragma once

#include "Simulation.hpp"

namespace traffic_flow {

struct TrafficState {
    float throughputAccumulator = 0.0f;
    std::size_t wrapCountAccumulator = 0;
    float queueAccumulator = 0.0f;
    float queueSamples = 0.0f;
    float elapsedTime = 0.0f;
};

class TrafficResetService {
public:
    static void clear(std::vector<Vehicle>& vehicles, SimulationStats& stats, TrafficState& state);
};

class TrafficSpawner {
public:
    static void generate(const Simulation& simulation, std::vector<Vehicle>& vehicles);
};

} // namespace traffic_flow
