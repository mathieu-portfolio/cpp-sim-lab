#pragma once

#include "Simulation.hpp"

#include <vector>

namespace traffic_flow_cpu_gpu {

class TrafficPhysics {
public:
    static bool canSpawnVehicleAt(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles,
        const Vehicle& candidate,
        float minimumCenterDistance);

    // Hard invariant pass. Behavior proposes speeds/positions, then physics
    // repairs them. Same-lane vehicles are solved as an ordered 1D chain along
    // the lane, so non-crossroad cars are clamped behind their leader instead of
    // being frozen by pairwise rejection artifacts. A final global footprint
    // projection pass then enforces the absolute invariant that no two vehicle
    // safety circles overlap, including at crossroads.
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

    static void enforceLaneOrderSpacing(
        const Simulation& simulation,
        const std::vector<Vehicle>& current,
        std::vector<Vehicle>& proposed,
        float minimumCenterDistance,
        float dt);

    static void enforceCrossroadFootprints(
        const Simulation& simulation,
        const std::vector<Vehicle>& current,
        std::vector<Vehicle>& proposed,
        float minimumCenterDistance,
        float dt);

    static void enforceGlobalFootprintProjection(
        const Simulation& simulation,
        const std::vector<Vehicle>& current,
        std::vector<Vehicle>& proposed,
        float minimumCenterDistance,
        float dt);
};

} // namespace traffic_flow_cpu_gpu
