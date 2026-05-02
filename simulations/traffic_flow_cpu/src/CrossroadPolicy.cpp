#include "CrossroadPolicy.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <cmath>

namespace traffic_flow_cpu {
namespace {

float dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

} // namespace

bool RightPriorityWithDeadlockBreakerPolicy::reservationMatchesVehicle(
    const CrossroadReservation& reservation,
    const Vehicle& vehicle) {
    return reservation.vehicleIndex >= 0 &&
        reservation.roadId == vehicle.roadId &&
        reservation.laneId == vehicle.laneId;
}

void RightPriorityWithDeadlockBreakerPolicy::reserveForVehicle(
    std::vector<CrossroadReservation>& reservations,
    std::size_t crossroadIndex,
    std::size_t vehicleIndex,
    const Vehicle& vehicle) {
    if (crossroadIndex >= reservations.size()) return;

    CrossroadReservation& reservation = reservations[crossroadIndex];
    if (reservation.vehicleIndex < 0) {
        reservation.vehicleIndex = static_cast<int>(vehicleIndex);
        reservation.roadId = vehicle.roadId;
        reservation.laneId = vehicle.laneId;
    }
}

std::vector<CrossroadReservation> RightPriorityWithDeadlockBreakerPolicy::rebuildReservations(
    const Simulation& simulation,
    const std::vector<Vehicle>& vehicles) const {
    std::vector<CrossroadReservation> reservations(simulation.getRoadNetwork().crossroads.size());

    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        const Vehicle& vehicle = vehicles[i];
        if (vehicle.reservedCrossroadId < 0) continue;

        const std::size_t reservedIndex = static_cast<std::size_t>(vehicle.reservedCrossroadId);
        if (reservedIndex >= reservations.size()) continue;

        std::size_t nearestIndex = 0;
        float approachS = 0.0f;
        float distance = 999999.0f;
        const bool stillApproachingReserved =
            simulation.findNearestCrossroadAhead(vehicle, nearestIndex, approachS, distance) &&
            nearestIndex == reservedIndex &&
            distance <= simulation.getConfig().crossroadReservationLookahead;

        if (vehicle.crossroadEngaged || simulation.isVehicleInsideCrossroad(vehicle, reservedIndex) || stillApproachingReserved) {
            const CrossroadReservation& reservation = reservations[reservedIndex];
            if (reservation.vehicleIndex < 0 || reservationMatchesVehicle(reservation, vehicle)) {
                reserveForVehicle(reservations, reservedIndex, i, vehicle);
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

    float ownApproachS = 0.0f;
    float ownDistance = 999999.0f;
    bool hasOwnApproach = false;
    for (const CrossroadApproach& approach : crossroad.approaches) {
        if (approach.roadId != vehicle.roadId) continue;
        const float distance = simulation.distanceToCrossroadAlongLane(vehicle, approach.s);
        if (distance < ownDistance) {
            ownDistance = distance;
            ownApproachS = approach.s;
            hasOwnApproach = true;
        }
    }

    if (!hasOwnApproach || ownDistance > config.crossroadYieldLookahead) return false;

    const Vec2 ownForward = simulation.sampleRoadTangent(vehicle.roadId, vehicle.laneId, ownApproachS);
    const Vec2 ownRight{ownForward.y, -ownForward.x};

    for (std::size_t otherIndex = 0; otherIndex < vehicles.size(); ++otherIndex) {
        if (otherIndex == vehicleIndex) continue;

        const Vehicle& other = vehicles[otherIndex];
        if (other.roadId >= network.roads.size()) continue;
        if (other.reservedCrossroadId == static_cast<int>(crossroadIndex) && other.crossroadEngaged) {
            continue; // already inside: handled by reservation/physics, not right priority
        }

        float otherApproachS = 0.0f;
        float otherDistance = 999999.0f;
        bool hasOtherApproach = false;
        for (const CrossroadApproach& approach : crossroad.approaches) {
            if (approach.roadId != other.roadId) continue;
            const float distance = simulation.distanceToCrossroadAlongLane(other, approach.s);
            if (distance < otherDistance) {
                otherDistance = distance;
                otherApproachS = approach.s;
                hasOtherApproach = true;
            }
        }

        if (!hasOtherApproach || otherDistance > config.crossroadYieldLookahead) continue;

        constexpr float movingSpeedEpsilon = 0.15f;
        if (other.speed <= movingSpeedEpsilon) continue;

        const Vec2 otherForward = simulation.sampleRoadTangent(other.roadId, other.laneId, otherApproachS);
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
        if (reservedIndex < reservations.size()) {
            reserveForVehicle(reservations, reservedIndex, vehicleIndex, vehicle);
            decision.reservedCrossroadId = static_cast<int>(reservedIndex);
            decision.ownsReservation = true;
            decision.shouldEngage = true;
            decision.isRelevant = true;
        }
        return decision;
    }

    if (!atReservationZone || nearestCrossroadIndex >= reservations.size()) {
        return decision;
    }

    decision.isRelevant = true;
    const CrossroadReservation& owner = reservations[nearestCrossroadIndex];
    const bool reservationIsFree = owner.vehicleIndex < 0;
    const bool sameReservedStream = reservationMatchesVehicle(owner, vehicle);

    if (!reservationIsFree && !sameReservedStream) {
        decision.shouldStop = true;
        return decision;
    }

    // Right priority gates reservation, even for following vehicles from the
    // currently reserved stream. Once a vehicle is engaged/inside, this method
    // has already returned above and priority no longer applies.
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

    reserveForVehicle(reservations, nearestCrossroadIndex, vehicleIndex, vehicle);
    decision.reservedCrossroadId = static_cast<int>(nearestCrossroadIndex);
    decision.ownsReservation = true;
    decision.shouldEngage = nearestCrossroadDistance <= config.crossroadStopRadius;
    return decision;
}

} // namespace traffic_flow_cpu
