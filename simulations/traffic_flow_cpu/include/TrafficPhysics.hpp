#pragma once

#include "Simulation.hpp"

#include <vector>

namespace traffic_flow_cpu {

class TrafficPhysics {
public:
    static bool canSpawnVehicleAt(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles,
        const Vehicle& candidate,
        float minimumCenterDistance);

    // Hard invariant pass. Vehicle behavior may propose any movement, but this
    // function is the final authority: a proposed position is accepted only if
    // its physical footprint stays clear of every already-reserved vehicle and
    // every not-yet-moved vehicle. Rejected vehicles remain where they were.
    static void enforceNoOverlap(
        const Simulation& simulation,
        const std::vector<Vehicle>& current,
        std::vector<Vehicle>& proposed,
        float minimumCenterDistance,
        float dt);

private:
    static bool overlapsAny(
        const Simulation& simulation,
        const Vehicle& candidate,
        const std::vector<Vehicle>& occupied,
        float minimumCenterDistance);
};

} // namespace traffic_flow_cpu
