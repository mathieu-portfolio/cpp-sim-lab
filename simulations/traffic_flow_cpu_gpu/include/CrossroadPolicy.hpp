#pragma once

#include "Vehicle.hpp"

#include <cstddef>
#include <vector>

namespace traffic_flow_cpu_gpu {

class Simulation;

struct MovementId {
    std::size_t crossroadId = static_cast<std::size_t>(-1);
    std::size_t roadId = static_cast<std::size_t>(-1);
    int laneId = -1;
    std::size_t entryApproachIndex = static_cast<std::size_t>(-1);
    std::size_t exitApproachIndex = static_cast<std::size_t>(-1);
};

struct CrossroadReservation {
    int vehicleIndex = -1;
    MovementId movement{};
};

struct CrossroadPolicyDecision {
    bool isRelevant = false;
    bool shouldStop = false;
    bool ownsReservation = false;
    bool shouldEngage = false;
    bool shouldRelease = false;
    bool sawMovingRightThreat = false;
    int reservedCrossroadId = -1;
};

class ICrossroadPolicy {
public:
    virtual ~ICrossroadPolicy() = default;

    virtual std::vector<CrossroadReservation> rebuildReservations(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles) const = 0;

    virtual CrossroadPolicyDecision decide(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles,
        std::size_t vehicleIndex,
        const Vehicle& vehicle,
        std::vector<CrossroadReservation>& reservations,
        float now) const = 0;
};

class RightPriorityWithDeadlockBreakerPolicy final : public ICrossroadPolicy {
public:
    std::vector<CrossroadReservation> rebuildReservations(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles) const override;

    CrossroadPolicyDecision decide(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles,
        std::size_t vehicleIndex,
        const Vehicle& vehicle,
        std::vector<CrossroadReservation>& reservations,
        float now) const override;

private:
    static bool sameMovement(const MovementId& lhs, const MovementId& rhs);
    static bool sameCrossroad(const MovementId& lhs, const MovementId& rhs);

    bool findMovementForVehicle(
        const Simulation& simulation,
        const Vehicle& vehicle,
        std::size_t crossroadIndex,
        MovementId& outMovement) const;

    bool movementsAreCompatible(
        const Simulation& simulation,
        const MovementId& lhs,
        const MovementId& rhs) const;

    bool movementIsActiveOrCompatible(
        const Simulation& simulation,
        const std::vector<CrossroadReservation>& reservations,
        const MovementId& movement) const;

    bool fairnessGateAllowsAdmission(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles,
        const std::vector<CrossroadReservation>& reservations,
        const MovementId& candidateMovement) const;

    static void reserveMovementForVehicle(
        std::vector<CrossroadReservation>& reservations,
        std::size_t vehicleIndex,
        const MovementId& movement);

    bool hasMovingRightSideThreatAtCrossroad(
        const Simulation& simulation,
        const std::vector<Vehicle>& vehicles,
        const Vehicle& vehicle,
        std::size_t vehicleIndex,
        std::size_t crossroadIndex) const;
};

} // namespace traffic_flow_cpu_gpu
