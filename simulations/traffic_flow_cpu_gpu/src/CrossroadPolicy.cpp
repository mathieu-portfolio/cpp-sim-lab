#include "CrossroadPolicy.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <cmath>

namespace traffic_flow_cpu_gpu {
namespace {

float dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

float cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

} // namespace

bool RightPriorityWithDeadlockBreakerPolicy::sameMovement(const MovementId& lhs, const MovementId& rhs) {
    return lhs.crossroadId == rhs.crossroadId &&
        lhs.roadId == rhs.roadId &&
        lhs.laneId == rhs.laneId &&
        lhs.entryApproachIndex == rhs.entryApproachIndex &&
        lhs.exitApproachIndex == rhs.exitApproachIndex;
}

bool RightPriorityWithDeadlockBreakerPolicy::sameCrossroad(const MovementId& lhs, const MovementId& rhs) {
    return lhs.crossroadId == rhs.crossroadId &&
        lhs.crossroadId != static_cast<std::size_t>(-1);
}

bool RightPriorityWithDeadlockBreakerPolicy::findMovementForVehicle(
    const Simulation& simulation,
    const Vehicle& vehicle,
    std::size_t crossroadIndex,
    MovementId& outMovement) const {
    const RoadNetwork& network = simulation.getRoadNetwork();
    if (crossroadIndex >= network.crossroads.size() || vehicle.roadId >= network.roads.size()) return false;

    const Crossroad& crossroad = network.crossroads[crossroadIndex];
    const RoadSegment& road = network.roads[vehicle.roadId];
    if (road.length <= 0.0f || vehicle.laneId < 0 || static_cast<std::size_t>(vehicle.laneId) >= road.lanes.size()) return false;

    float nearestDistance = 999999.0f;
    std::size_t entryApproach = static_cast<std::size_t>(-1);
    for (std::size_t i = 0; i < crossroad.approaches.size(); ++i) {
        const CrossroadApproach& approach = crossroad.approaches[i];
        if (approach.roadId != vehicle.roadId) continue;

        const float distance = simulation.distanceToCrossroadAlongLane(vehicle, approach.s);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            entryApproach = i;
        }
    }

    if (entryApproach == static_cast<std::size_t>(-1)) return false;

    // Future-ready field: for now vehicles continue on their current loop/lane.
    // Pick the approach whose outgoing tangent best matches the current forward
    // direction as the notional exit arm. Turning policies can replace this later.
    const Vec2 entryForward = simulation.sampleRoadTangent(vehicle.roadId, vehicle.laneId, crossroad.approaches[entryApproach].s);
    float bestExitAlignment = -999999.0f;
    std::size_t exitApproach = entryApproach;
    for (std::size_t i = 0; i < crossroad.approaches.size(); ++i) {
        if (i == entryApproach) continue;
        const CrossroadApproach& approach = crossroad.approaches[i];
        if (approach.roadId != vehicle.roadId) continue;

        const Vec2 candidateForward = simulation.sampleRoadTangent(vehicle.roadId, vehicle.laneId, approach.s);
        const float alignment = dot(entryForward, candidateForward);
        if (alignment > bestExitAlignment) {
            bestExitAlignment = alignment;
            exitApproach = i;
        }
    }

    outMovement.crossroadId = crossroadIndex;
    outMovement.roadId = vehicle.roadId;
    outMovement.laneId = vehicle.laneId;
    outMovement.entryApproachIndex = entryApproach;
    outMovement.exitApproachIndex = exitApproach;
    return true;
}

bool RightPriorityWithDeadlockBreakerPolicy::movementsAreCompatible(
    const Simulation& simulation,
    const MovementId& lhs,
    const MovementId& rhs) const {
    if (!sameCrossroad(lhs, rhs)) return true;
    if (sameMovement(lhs, rhs)) return true;

    const RoadNetwork& network = simulation.getRoadNetwork();
    if (lhs.crossroadId >= network.crossroads.size()) return false;
    const Crossroad& crossroad = network.crossroads[lhs.crossroadId];
    if (lhs.entryApproachIndex >= crossroad.approaches.size() || rhs.entryApproachIndex >= crossroad.approaches.size()) return false;

    const CrossroadApproach& lhsEntry = crossroad.approaches[lhs.entryApproachIndex];
    const CrossroadApproach& rhsEntry = crossroad.approaches[rhs.entryApproachIndex];
    if (lhsEntry.roadId >= network.roads.size() || rhsEntry.roadId >= network.roads.size()) return false;

    const Vec2 lhsForward = simulation.sampleRoadTangent(lhsEntry.roadId, lhs.laneId, lhsEntry.s);
    const Vec2 rhsForward = simulation.sampleRoadTangent(rhsEntry.roadId, rhs.laneId, rhsEntry.s);

    const float alignment = dot(lhsForward, rhsForward);
    const float crossing = std::fabs(cross(lhsForward, rhsForward));

    // Allow opposite straight-through streams to share the crossing. This is the
    // first compatible-stream rule; turn-specific conflict corridors can replace
    // it later without changing the vehicle/physics layers.
    if (alignment < -0.78f && crossing < 0.65f) {
        return true;
    }

    return false;
}

bool RightPriorityWithDeadlockBreakerPolicy::movementIsActiveOrCompatible(
    const Simulation& simulation,
    const std::vector<CrossroadReservation>& reservations,
    const MovementId& movement) const {
    for (const CrossroadReservation& reservation : reservations) {
        if (reservation.vehicleIndex < 0) continue;
        if (!sameCrossroad(reservation.movement, movement)) continue;
        if (!movementsAreCompatible(simulation, reservation.movement, movement)) {
            return false;
        }
    }
    return true;
}

bool RightPriorityWithDeadlockBreakerPolicy::fairnessGateAllowsAdmission(
    const Simulation& simulation,
    const std::vector<Vehicle>& vehicles,
    const std::vector<CrossroadReservation>& reservations,
    const MovementId& candidateMovement) const {
    const SimulationConfig& config = simulation.getConfig();
    const RoadNetwork& network = simulation.getRoadNetwork();
    if (candidateMovement.crossroadId >= network.crossroads.size()) return true;

    // Same/compatible movements may platoon, but not forever. If a conflicting
    // movement has waited too long, stop admitting new vehicles from the active
    // platoon and let the crossroad clear for the longest waiting movement.
    float longestConflictingWait = 0.0f;
    MovementId longestWaitingMovement{};
    bool hasLongWaitingConflict = false;

    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        const Vehicle& other = vehicles[i];
        if (other.crossroadEngaged) continue;

        std::size_t nearestCrossroad = 0;
        float approachS = 0.0f;
        float distance = 999999.0f;
        if (!simulation.findNearestCrossroadAhead(other, nearestCrossroad, approachS, distance)) continue;
        if (nearestCrossroad != candidateMovement.crossroadId) continue;
        if (distance > config.crossroadReservationLookahead) continue;

        MovementId otherMovement{};
        if (!findMovementForVehicle(simulation, other, nearestCrossroad, otherMovement)) continue;
        if (movementsAreCompatible(simulation, candidateMovement, otherMovement)) continue;
        if (other.crossroadWaitTime < config.crossroadDeadlockBreakerWait) continue;

        if (!hasLongWaitingConflict || other.crossroadWaitTime > longestConflictingWait) {
            hasLongWaitingConflict = true;
            longestConflictingWait = other.crossroadWaitTime;
            longestWaitingMovement = otherMovement;
        }
    }

    if (!hasLongWaitingConflict) return true;

    // If this candidate is the long-waiting conflicting movement, allow it only
    // once incompatible active movements have cleared. Otherwise close the gate.
    if (sameMovement(candidateMovement, longestWaitingMovement)) {
        for (const CrossroadReservation& reservation : reservations) {
            if (reservation.vehicleIndex < 0) continue;
            if (!sameCrossroad(reservation.movement, candidateMovement)) continue;
            if (!movementsAreCompatible(simulation, reservation.movement, candidateMovement)) {
                return false;
            }
        }
        return true;
    }

    return false;
}

void RightPriorityWithDeadlockBreakerPolicy::reserveMovementForVehicle(
    std::vector<CrossroadReservation>& reservations,
    std::size_t vehicleIndex,
    const MovementId& movement) {
    for (CrossroadReservation& reservation : reservations) {
        if (reservation.vehicleIndex == static_cast<int>(vehicleIndex)) {
            reservation.movement = movement;
            return;
        }
    }

    reservations.push_back({static_cast<int>(vehicleIndex), movement});
}

std::vector<CrossroadReservation> RightPriorityWithDeadlockBreakerPolicy::rebuildReservations(
    const Simulation& simulation,
    const std::vector<Vehicle>& vehicles) const {
    std::vector<CrossroadReservation> reservations;
    reservations.reserve(vehicles.size());

    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        const Vehicle& vehicle = vehicles[i];
        if (vehicle.reservedCrossroadId < 0) continue;

        const std::size_t reservedIndex = static_cast<std::size_t>(vehicle.reservedCrossroadId);
        if (reservedIndex >= simulation.getRoadNetwork().crossroads.size()) continue;

        std::size_t nearestIndex = 0;
        float approachS = 0.0f;
        float distance = 999999.0f;
        const bool stillApproachingReserved =
            simulation.findNearestCrossroadAhead(vehicle, nearestIndex, approachS, distance) &&
            nearestIndex == reservedIndex &&
            distance <= simulation.getConfig().crossroadReservationLookahead;

        if (vehicle.crossroadEngaged || simulation.isVehicleInsideCrossroad(vehicle, reservedIndex) || stillApproachingReserved) {
            MovementId movement{};
            if (findMovementForVehicle(simulation, vehicle, reservedIndex, movement)) {
                reserveMovementForVehicle(reservations, i, movement);
            }
        }
    }

    return reservations;
}

bool RightPriorityWithDeadlockBreakerPolicy::hasMovingRightSideThreatAtCrossroad(
    const Simulation& simulation,
    const std::vector<Vehicle>& vehicles,
    const Vehicle& vehicle,
    std::size_t vehicleIndex,
    std::size_t crossroadIndex) const {
    const RoadNetwork& network = simulation.getRoadNetwork();
    if (crossroadIndex >= network.crossroads.size()) return false;

    const SimulationConfig& config = simulation.getConfig();
    const Crossroad& crossroad = network.crossroads[crossroadIndex];

    MovementId ownMovement{};
    if (!findMovementForVehicle(simulation, vehicle, crossroadIndex, ownMovement)) return false;
    if (ownMovement.entryApproachIndex >= crossroad.approaches.size()) return false;

    const CrossroadApproach& ownApproach = crossroad.approaches[ownMovement.entryApproachIndex];
    const float ownDistance = simulation.distanceToCrossroadAlongLane(vehicle, ownApproach.s);
    if (ownDistance > config.crossroadYieldLookahead) return false;

    const Vec2 ownForward = simulation.sampleRoadTangent(vehicle.roadId, vehicle.laneId, ownApproach.s);
    const Vec2 ownRight{ownForward.y, -ownForward.x};

    for (std::size_t otherIndex = 0; otherIndex < vehicles.size(); ++otherIndex) {
        if (otherIndex == vehicleIndex) continue;

        const Vehicle& other = vehicles[otherIndex];
        if (other.roadId >= network.roads.size()) continue;
        if (other.reservedCrossroadId == static_cast<int>(crossroadIndex) && other.crossroadEngaged) {
            continue; // already inside: handled by reservation/physics, not right priority
        }

        MovementId otherMovement{};
        if (!findMovementForVehicle(simulation, other, crossroadIndex, otherMovement)) continue;
        if (otherMovement.entryApproachIndex >= crossroad.approaches.size()) continue;

        const CrossroadApproach& otherApproach = crossroad.approaches[otherMovement.entryApproachIndex];
        const float otherDistance = simulation.distanceToCrossroadAlongLane(other, otherApproach.s);
        if (otherDistance > config.crossroadYieldLookahead) continue;

        constexpr float movingSpeedEpsilon = 0.15f;
        if (other.speed <= movingSpeedEpsilon) continue;

        const Vec2 otherForward = simulation.sampleRoadTangent(other.roadId, other.laneId, otherApproach.s);
        const float rightness = dot(ownRight, otherForward);
        const float parallel = std::fabs(dot(ownForward, otherForward));
        if (rightness > 0.20f && parallel < 0.92f) {
            return true;
        }
    }

    return false;
}

CrossroadPolicyDecision RightPriorityWithDeadlockBreakerPolicy::decide(
    const Simulation& simulation,
    const std::vector<Vehicle>& vehicles,
    std::size_t vehicleIndex,
    const Vehicle& vehicle,
    std::vector<CrossroadReservation>& reservations,
    float now) const {
    CrossroadPolicyDecision decision;

    const RoadNetwork& network = simulation.getRoadNetwork();
    const SimulationConfig& config = simulation.getConfig();
    if (vehicle.roadId >= network.roads.size()) return decision;

    std::size_t nearestCrossroadIndex = 0;
    float nearestApproachS = 0.0f;
    float nearestCrossroadDistance = 999999.0f;
    const bool hasNearestCrossroad = simulation.findNearestCrossroadAhead(
        vehicle,
        nearestCrossroadIndex,
        nearestApproachS,
        nearestCrossroadDistance);
    const bool atReservationZone = hasNearestCrossroad &&
        nearestCrossroadDistance <= config.crossroadReservationLookahead;

    const bool insideReservedCrossroad = vehicle.reservedCrossroadId >= 0 &&
        simulation.isVehicleInsideCrossroad(vehicle, static_cast<std::size_t>(vehicle.reservedCrossroadId));

    if (vehicle.reservedCrossroadId >= 0 && !insideReservedCrossroad && !atReservationZone) {
        decision.shouldRelease = true;
        return decision;
    }

    if (vehicle.crossroadEngaged || insideReservedCrossroad) {
        const std::size_t reservedIndex = vehicle.reservedCrossroadId >= 0
            ? static_cast<std::size_t>(vehicle.reservedCrossroadId)
            : nearestCrossroadIndex;
        if (reservedIndex < network.crossroads.size()) {
            MovementId movement{};
            if (findMovementForVehicle(simulation, vehicle, reservedIndex, movement)) {
                reserveMovementForVehicle(reservations, vehicleIndex, movement);
                decision.reservedCrossroadId = static_cast<int>(reservedIndex);
                decision.ownsReservation = true;
                decision.shouldEngage = true;
                decision.isRelevant = true;
            }
        }
        return decision;
    }

    if (!atReservationZone || nearestCrossroadIndex >= network.crossroads.size()) {
        return decision;
    }

    decision.isRelevant = true;

    MovementId candidateMovement{};
    if (!findMovementForVehicle(simulation, vehicle, nearestCrossroadIndex, candidateMovement)) {
        return decision;
    }

    // Right priority gates reservation. Reservation/compatibility should never
    // grant entry to a vehicle that still owes priority to a moving car on the right.
    const bool movingRightThreat = hasMovingRightSideThreatAtCrossroad(
        simulation,
        vehicles,
        vehicle,
        vehicleIndex,
        nearestCrossroadIndex);
    const bool stoppedRightGrace = now - vehicle.lastMovingRightPriorityTime < config.stoppedRightPriorityGrace;
    const bool deadlockBreakerActive = vehicle.crossroadWaitTime >= config.crossroadDeadlockBreakerWait;

    if (!deadlockBreakerActive && (movingRightThreat || stoppedRightGrace)) {
        decision.shouldStop = true;
        decision.sawMovingRightThreat = movingRightThreat;
        return decision;
    }

    if (!movementIsActiveOrCompatible(simulation, reservations, candidateMovement)) {
        decision.shouldStop = true;
        return decision;
    }

    if (!fairnessGateAllowsAdmission(simulation, vehicles, reservations, candidateMovement)) {
        decision.shouldStop = true;
        return decision;
    }

    reserveMovementForVehicle(reservations, vehicleIndex, candidateMovement);
    decision.reservedCrossroadId = static_cast<int>(nearestCrossroadIndex);
    decision.ownsReservation = true;
    decision.shouldEngage = nearestCrossroadDistance <= config.crossroadStopRadius;
    return decision;
}

} // namespace traffic_flow_cpu_gpu
