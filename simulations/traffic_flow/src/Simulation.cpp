#include "Simulation.hpp"
#include "CrossroadPolicy.hpp"
#include "TrafficPhysics.hpp"
#include "VehicleBehavior.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

#include <utility>

namespace traffic_flow {
namespace {

Vec2 catmullRom(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

int wrapIndex(int index, int count) {
    return (index % count + count) % count;
}

float wrapDistance(float s, float length) {
    if (length <= 0.0f) return 0.0f;
    float wrapped = std::fmod(s, length);
    if (wrapped < 0.0f) wrapped += length;
    return wrapped;
}

float cross(const Vec2& a, const Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

float dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

bool lineIntersection(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, float& outU, float& outV) {
    const Vec2 r = b - a;
    const Vec2 s = d - c;
    const float denom = cross(r, s);
    if (std::fabs(denom) < 1e-4f) return false;

    const Vec2 delta = c - a;
    outU = cross(delta, s) / denom;
    outV = cross(delta, r) / denom;
    return outU > 0.02f && outU < 0.98f && outV > 0.02f && outV < 0.98f;
}

std::vector<Vec2> buildDrivePoints(const std::vector<Vec2>& rawPoints, float /*smoothingIterations*/, float minPointDistance) {
    if (rawPoints.size() < 2) return rawPoints;

    const Vec2 start = rawPoints.front();
    const Vec2 end = rawPoints.back();
    const float length = (end - start).length();
    const float step = std::max(4.0f, minPointDistance);
    const int subdivisions = std::max(1, static_cast<int>(std::round(length / step)));

    std::vector<Vec2> points;
    points.reserve(static_cast<std::size_t>(subdivisions + 1));
    for (int i = 0; i <= subdivisions; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(subdivisions);
        points.push_back(start + (end - start) * t);
    }
    return points;
}

Vec2 sampleByT(const RoadSegment& road, float t) {
    const std::vector<Vec2>& points = road.drivePoints.empty() ? road.controlPoints : road.drivePoints;
    if (points.empty()) return {};
    if (points.size() == 1) return points.front();

    const float clampedT = std::clamp(t, 0.0f, 1.0f);
    const float scaled = clampedT * static_cast<float>(points.size() - 1);
    const int lo = static_cast<int>(std::floor(scaled));
    const int hi = std::min(static_cast<int>(points.size() - 1), lo + 1);
    const float alpha = scaled - static_cast<float>(lo);
    return points[static_cast<std::size_t>(lo)] +
           (points[static_cast<std::size_t>(hi)] - points[static_cast<std::size_t>(lo)]) * alpha;
}

} // namespace

Simulation::Simulation(SimulationConfig config) : m_config(config) {
    sanitizeConfig();
    resetDefaultRoad();
    generateTraffic();
}

void Simulation::sanitizeConfig() {
    m_config.vehicleCount = std::clamp<std::size_t>(m_config.vehicleCount, 0, 500);
    m_config.desiredSpeed = std::clamp(m_config.desiredSpeed, 1.0f, 60.0f);
    m_config.maxAcceleration = std::clamp(m_config.maxAcceleration, 0.0f, 4.0f);
    m_config.comfortableBraking = std::clamp(m_config.comfortableBraking, 0.1f, 5.0f);
    m_config.minimumGap = std::clamp(m_config.minimumGap, 0.0f, 20.0f);
    m_config.desiredTimeHeadway = std::clamp(m_config.desiredTimeHeadway, 0.1f, 4.0f);
    m_config.laneWidth = std::clamp(m_config.laneWidth, 4.0f, 80.0f);
    m_config.spawnSpeedMin = std::clamp(m_config.spawnSpeedMin, 0.0f, 60.0f);
    m_config.spawnSpeedMax = std::clamp(m_config.spawnSpeedMax, m_config.spawnSpeedMin, 80.0f);
    m_config.crossroadYieldLookahead = std::clamp(m_config.crossroadYieldLookahead, 15.0f, 120.0f);
    m_config.crossroadStopRadius = std::clamp(m_config.crossroadStopRadius, 4.0f, 30.0f);
    m_config.crossroadClearDelay = std::clamp(m_config.crossroadClearDelay, 0.1f, 2.0f);
    m_config.stoppedRightPriorityGrace = std::clamp(m_config.stoppedRightPriorityGrace, 0.0f, 2.0f);
    m_config.crossroadDeadlockBreakerWait = std::clamp(m_config.crossroadDeadlockBreakerWait, 0.25f, 8.0f);
    m_config.crossroadReservationLookahead = std::clamp(m_config.crossroadReservationLookahead, 15.0f, 140.0f);
    m_config.spawnCrossroadClearance = std::clamp(m_config.spawnCrossroadClearance, 0.0f, 100.0f);
    m_config.spawnMinimumGap = std::clamp(m_config.spawnMinimumGap, 6.0f, 60.0f);
    m_config.physicsMinimumGap = std::clamp(m_config.physicsMinimumGap, 0.0f, 20.0f);
    m_config.roadSmoothingIterations = std::clamp(m_config.roadSmoothingIterations, 0.0f, 5.0f);
    m_config.roadSmoothingMinPointDistance = std::clamp(m_config.roadSmoothingMinPointDistance, 2.0f, 30.0f);
    m_config.arcLengthSamplesPerSpan = std::clamp(m_config.arcLengthSamplesPerSpan, 8, 96);
}

void Simulation::resetDefaultRoad() {
    sanitizeConfig();
    RoadSegment road;
    road.controlPoints = {{120.0f, 350.0f}, {980.0f, 350.0f}};
    road.lanes = {{1, -m_config.laneWidth * 0.5f}, {-1, m_config.laneWidth * 0.5f}};
    rebuildRoadCache(road);
    m_network.roads = {road};
    rebuildCrossroads();
}

void Simulation::rebuildRoadCaches() {
    for (RoadSegment& road : m_network.roads) {
        rebuildRoadCache(road);
    }
    rebuildCrossroads();
}

void Simulation::rebuildRoadCache(RoadSegment& road) {
    sanitizeConfig();
    road.drivePoints = buildDrivePoints(
        road.controlPoints,
        m_config.roadSmoothingIterations,
        m_config.roadSmoothingMinPointDistance);

    const int spans = std::max(1, static_cast<int>(road.drivePoints.empty() ? road.controlPoints.size() : road.drivePoints.size()));
    const int samples = std::max(2, spans * std::max(2, m_config.arcLengthSamplesPerSpan));
    road.arcLengthCache.assign(static_cast<std::size_t>(samples + 1), 0.0f);
    Vec2 prev = sampleByT(road, 0.0f);
    float len = 0.0f;
    for (int i = 1; i <= samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples);
        const Vec2 p = sampleByT(road, t);
        len += (p - prev).length();
        road.arcLengthCache[static_cast<std::size_t>(i)] = len;
        prev = p;
    }
    road.length = len;
}

void Simulation::rebuildCrossroads() {
    m_network.crossroads.clear();

    for (std::size_t roadId = 0; roadId < m_network.roads.size(); ++roadId) {
        const RoadSegment& road = m_network.roads[roadId];
        if (road.length <= 1.0f || road.arcLengthCache.size() < 8) continue;

        const int samples = std::max(64, static_cast<int>(road.drivePoints.empty() ? road.controlPoints.size() : road.drivePoints.size()) * std::max(12, m_config.arcLengthSamplesPerSpan));
        struct Sample { Vec2 p; float s; };
        std::vector<Sample> centerline;
        centerline.reserve(static_cast<std::size_t>(samples + 1));
        for (int i = 0; i <= samples; ++i) {
            const float s = road.length * static_cast<float>(i) / static_cast<float>(samples);
            centerline.push_back({sampleRoadCenter(roadId, s), s});
        }

        for (int a = 0; a < samples; ++a) {
            for (int b = a + 2; b < samples; ++b) {
                if (a == 0 && b == samples - 1) continue; // same closed-loop seam

                float u = 0.0f;
                float v = 0.0f;
                if (!lineIntersection(centerline[a].p, centerline[a + 1].p, centerline[b].p, centerline[b + 1].p, u, v)) continue;

                const float sA = centerline[a].s + (centerline[a + 1].s - centerline[a].s) * u;
                const float sB = centerline[b].s + (centerline[b + 1].s - centerline[b].s) * v;
                if (std::fabs(sA - sB) < m_config.crossroadYieldLookahead) continue;
                if (road.length - std::fabs(sA - sB) < m_config.crossroadYieldLookahead) continue;

                const Vec2 pos = centerline[a].p + (centerline[a + 1].p - centerline[a].p) * u;

                bool merged = false;
                for (Crossroad& existing : m_network.crossroads) {
                    if ((existing.position - pos).lengthSquared() < 28.0f * 28.0f) {
                        existing.approaches.push_back({roadId, wrapDistance(sA, road.length)});
                        existing.approaches.push_back({roadId, wrapDistance(sB, road.length)});
                        merged = true;
                        break;
                    }
                }
                if (!merged) {
                    Crossroad crossroad;
                    crossroad.position = pos;
                    crossroad.approaches.push_back({roadId, wrapDistance(sA, road.length)});
                    crossroad.approaches.push_back({roadId, wrapDistance(sB, road.length)});
                    m_network.crossroads.push_back(std::move(crossroad));
                }
            }
        }
    }
}

Vec2 Simulation::sampleRoadCenter(std::size_t roadId, float s) const {
    const RoadSegment& road = m_network.roads[roadId];
    if (road.length <= 0.0f || road.arcLengthCache.size() < 2) return sampleByT(road, 0.0f);
    const float clampedS = wrapDistance(s, road.length);
    const auto it = std::lower_bound(road.arcLengthCache.begin(), road.arcLengthCache.end(), clampedS);
    const std::size_t hi = static_cast<std::size_t>(std::distance(road.arcLengthCache.begin(), it));
    if (hi == 0) return sampleByT(road, 0.0f);
    const std::size_t lo = hi - 1;
    const float s0 = road.arcLengthCache[lo];
    const float s1 = road.arcLengthCache[hi];
    const float alpha = (s1 > s0) ? (clampedS - s0) / (s1 - s0) : 0.0f;
    const float t0 = static_cast<float>(lo) / static_cast<float>(road.arcLengthCache.size() - 1);
    const float t1 = static_cast<float>(hi) / static_cast<float>(road.arcLengthCache.size() - 1);
    return sampleByT(road, t0 + (t1 - t0) * alpha);
}

Vec2 Simulation::sampleRoadTangent(std::size_t roadId, int laneId, float s) const {
    const RoadSegment& road = m_network.roads[roadId];
    if (laneId < 0 || static_cast<std::size_t>(laneId) >= road.lanes.size()) {
        return {1.0f, 0.0f};
    }
    const float eps = 0.5f;
    const Vec2 p0 = sampleRoadCenter(roadId, s - eps);
    const Vec2 p1 = sampleRoadCenter(roadId, s + eps);
    Vec2 tangent = p1 - p0;
    if (tangent.lengthSquared() < 1e-6f) tangent = {1.0f, 0.0f};
    else tangent = tangent.normalized();
    if (road.lanes[static_cast<std::size_t>(laneId)].direction < 0) tangent = tangent * -1.0f;
    return tangent;
}

Vec2 Simulation::sampleLanePosition(std::size_t roadId, int laneId, float s) const {
    const RoadSegment& road = m_network.roads[roadId];
    if (laneId < 0 || static_cast<std::size_t>(laneId) >= road.lanes.size()) {
        return sampleRoadCenter(roadId, s);
    }
    const Vec2 c = sampleRoadCenter(roadId, s);

    // Use the centerline tangent for lane offsets. sampleRoadTangent() includes
    // the lane travel direction, so the opposite-direction lane was being flipped
    // back onto the same physical side of the road.
    const float eps = 0.5f;
    const Vec2 p0 = sampleRoadCenter(roadId, s - eps);
    const Vec2 p1 = sampleRoadCenter(roadId, s + eps);
    Vec2 centerTangent = p1 - p0;
    if (centerTangent.lengthSquared() < 1e-6f) centerTangent = {1.0f, 0.0f};
    else centerTangent = centerTangent.normalized();

    const Vec2 leftNormal{-centerTangent.y, centerTangent.x};
    return c + leftNormal * road.lanes[static_cast<std::size_t>(laneId)].lateralOffset;
}

float Simulation::idmAcceleration(const Vehicle& vehicle, const Vehicle* leader, float gap) const {
    const float freeRoadTerm = std::pow(std::max(0.0f, vehicle.speed / std::max(1.0f, m_config.desiredSpeed)), m_config.accelerationExponent);
    float interactionTerm = 0.0f;
    if (leader != nullptr) {
        const float deltaV = vehicle.speed - leader->speed;
        const float sqrtTerm = std::max(0.1f, 2.0f * std::sqrt(std::max(0.0f, m_config.maxAcceleration * m_config.comfortableBraking)));
        const float desiredGap = m_config.minimumGap + vehicle.speed * m_config.desiredTimeHeadway + std::max(0.0f, vehicle.speed * deltaV / sqrtTerm);
        const float ratio = desiredGap / std::max(0.5f, gap);
        interactionTerm = ratio * ratio;
    }
    return m_config.maxAcceleration * (1.0f - freeRoadTerm - interactionTerm);
}

void Simulation::reset() {
    resetDefaultRoad();
    generateTraffic();
}

void Simulation::clearTraffic() {
    m_vehicles.clear();
    m_throughputAccumulator = 0.0f;
    m_wrapCountAccumulator = 0;
    m_queueAccumulator = 0.0f;
    m_queueSamples = 0.0f;
    m_stats = {};
    m_elapsedTime = 0.0f;
}

void Simulation::notifyRoadsEdited() {
    rebuildRoadCaches();
    clearTraffic();
}

void Simulation::generateTraffic() {
    sanitizeConfig();
    clearTraffic();

    std::vector<std::pair<std::size_t, int>> spawnLanes;
    for (std::size_t roadId = 0; roadId < m_network.roads.size(); ++roadId) {
        const RoadSegment& road = m_network.roads[roadId];
        if (road.length <= 1.0f || road.lanes.empty()) continue;
        for (std::size_t laneId = 0; laneId < road.lanes.size(); ++laneId) {
            spawnLanes.emplace_back(roadId, static_cast<int>(laneId));
        }
    }

    if (spawnLanes.empty()) return;

    const std::size_t laneCount = spawnLanes.size();
    std::vector<std::size_t> perLaneTarget(laneCount, 0);
    for (std::size_t i = 0; i < m_config.vehicleCount; ++i) {
        ++perLaneTarget[i % laneCount];
    }

    for (std::size_t laneSlot = 0; laneSlot < laneCount; ++laneSlot) {
        const auto& laneRef = spawnLanes[laneSlot];
        const RoadSegment& road = m_network.roads[laneRef.first];
        const std::size_t targetCount = perLaneTarget[laneSlot];
        if (targetCount == 0 || road.length <= 1.0f) continue;

        const float spacing = std::max(m_config.spawnMinimumGap, m_config.minimumGap + 4.5f);
        const std::size_t maxByLength = static_cast<std::size_t>(std::max(1.0f, std::floor(road.length / spacing)));
        const std::size_t count = std::min(targetCount, maxByLength);

        std::vector<float> placedS;
        placedS.reserve(count);

        // Walk the whole lane at the requested spacing and skip only the
        // forbidden crossroad ranges. Do not stop generation when a crossroad
        // is met: traffic resumes automatically on the next regular-road
        // portion.
        auto tryPlace = [&](float s) {
            if (isInsideCrossroadSpawnClearance(laneRef.first, s)) return;

            Vehicle candidate;
            candidate.roadId = laneRef.first;
            candidate.laneId = laneRef.second;
            candidate.s = s;
            candidate.length = 4.5f;

            std::vector<Vehicle> alreadyPlaced = m_vehicles;
            alreadyPlaced.reserve(m_vehicles.size() + placedS.size());
            for (float existingS : placedS) {
                Vehicle placed;
                placed.roadId = laneRef.first;
                placed.laneId = laneRef.second;
                placed.s = existingS;
                placed.length = candidate.length;
                alreadyPlaced.push_back(placed);
            }

            if (TrafficPhysics::canSpawnVehicleAt(*this, alreadyPlaced, candidate, spacing)) {
                placedS.push_back(s);
            }
        };

        for (float s = spacing * 0.5f; s < road.length && placedS.size() < count; s += spacing) {
            tryPlace(s);
        }

        // Second pass with a different phase to fill valid road portions that
        // were skipped because the first phase landed inside crossroad zones.
        for (float s = spacing; s < road.length && placedS.size() < count; s += spacing) {
            tryPlace(s);
        }

        for (float s : placedS) {
            Vehicle v;
            v.roadId = laneRef.first;
            v.laneId = laneRef.second;
            v.s = s;
            v.speed = Random::range(m_config.spawnSpeedMin, m_config.spawnSpeedMax);
            m_vehicles.push_back(v);
        }
    }
}

float Simulation::distanceToCrossroadAlongLane(const Vehicle& vehicle, float crossroadS) const {
    const RoadSegment& road = m_network.roads[vehicle.roadId];
    if (road.length <= 0.0f) return 999999.0f;

    const int dir = road.lanes[static_cast<std::size_t>(vehicle.laneId)].direction;
    if (dir >= 0) return wrapDistance(crossroadS - vehicle.s, road.length);
    return wrapDistance(vehicle.s - crossroadS, road.length);
}


bool Simulation::isInsideCrossroadSpawnClearance(std::size_t roadId, float s) const {
    if (roadId >= m_network.roads.size()) return false;
    const RoadSegment& road = m_network.roads[roadId];
    if (road.length <= 0.0f) return false;

    for (const Crossroad& crossroad : m_network.crossroads) {
        for (const CrossroadApproach& approach : crossroad.approaches) {
            if (approach.roadId != roadId) continue;
            const float forward = wrapDistance(s - approach.s, road.length);
            const float backward = wrapDistance(approach.s - s, road.length);
            if (std::min(forward, backward) < m_config.spawnCrossroadClearance) {
                return true;
            }
        }
    }
    return false;
}

float Simulation::distanceAheadOnLane(const Vehicle& follower, const Vehicle& leader) const {
    if (follower.roadId != leader.roadId || follower.laneId != leader.laneId) return 999999.0f;

    const RoadSegment& road = m_network.roads[follower.roadId];
    if (road.length <= 0.0f) return 999999.0f;

    const int dir = road.lanes[static_cast<std::size_t>(follower.laneId)].direction;
    if (dir >= 0) return wrapDistance(leader.s - follower.s, road.length);
    return wrapDistance(follower.s - leader.s, road.length);
}

const Vehicle* Simulation::findLeader(const Vehicle& vehicle, std::size_t vehicleIndex, float& outGap) const {
    const Vehicle* leader = nullptr;
    outGap = 999999.0f;

    for (std::size_t otherIndex = 0; otherIndex < m_vehicles.size(); ++otherIndex) {
        if (otherIndex == vehicleIndex) continue;

        const Vehicle& other = m_vehicles[otherIndex];
        const float centerDistance = distanceAheadOnLane(vehicle, other);
        if (centerDistance <= 0.01f || centerDistance >= outGap) continue;

        const float bumperGap = centerDistance - (vehicle.length + other.length) * 0.5f;
        if (bumperGap < outGap) {
            outGap = bumperGap;
            leader = &other;
        }
    }

    return leader;
}

bool Simulation::findNearestCrossroadAhead(
    const Vehicle& vehicle,
    std::size_t& outCrossroadIndex,
    float& outApproachS,
    float& outDistance) const {
    outCrossroadIndex = static_cast<std::size_t>(-1);
    outApproachS = 0.0f;
    outDistance = 999999.0f;

    if (vehicle.roadId >= m_network.roads.size()) return false;

    for (std::size_t crossroadIndex = 0; crossroadIndex < m_network.crossroads.size(); ++crossroadIndex) {
        const Crossroad& crossroad = m_network.crossroads[crossroadIndex];
        for (const CrossroadApproach& approach : crossroad.approaches) {
            if (approach.roadId != vehicle.roadId) continue;
            const float distance = distanceToCrossroadAlongLane(vehicle, approach.s);
            if (distance < outDistance) {
                outDistance = distance;
                outApproachS = approach.s;
                outCrossroadIndex = crossroadIndex;
            }
        }
    }

    return outCrossroadIndex != static_cast<std::size_t>(-1);
}

bool Simulation::isVehicleInsideCrossroad(const Vehicle& vehicle, std::size_t crossroadIndex) const {
    if (vehicle.roadId >= m_network.roads.size()) return false;
    if (crossroadIndex >= m_network.crossroads.size()) return false;

    const RoadSegment& road = m_network.roads[vehicle.roadId];
    if (road.length <= 0.0f) return false;

    for (const CrossroadApproach& approach : m_network.crossroads[crossroadIndex].approaches) {
        if (approach.roadId != vehicle.roadId) continue;
        const float distance = distanceToCrossroadAlongLane(vehicle, approach.s);
        if (distance <= m_config.crossroadStopRadius ||
            distance >= road.length - m_config.crossroadStopRadius) {
            return true;
        }
    }

    return false;
}


void Simulation::update(float dt) {
    sanitizeConfig();
    m_elapsedTime += dt;

    RightPriorityWithDeadlockBreakerPolicy crossroadPolicy;
    DefaultVehicleBehavior vehicleBehavior;
    std::vector<CrossroadReservation> crossroadReservations =
        crossroadPolicy.rebuildReservations(*this, m_vehicles);

    const auto backend = m_config.execution.computeBackend;
    const bool useGpuComputePath = backend == simfw::simulation::ComputeBackend::GpuCompute;

    std::vector<Vehicle> next = m_vehicles;
    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        Vehicle& v = next[i];
        if (v.roadId >= m_network.roads.size()) continue;

        const RoadSegment& road = m_network.roads[v.roadId];
        if (road.length <= 0.0f || v.laneId < 0 || static_cast<std::size_t>(v.laneId) >= road.lanes.size()) {
            continue;
        }

        // GPU path toggle point: crossroad policy evaluation is isolated here so
        // a future compute kernel can replace this call without touching the
        // surrounding state transitions.
        const CrossroadPolicyDecision crossroadDecision = crossroadPolicy.decide(
            *this,
            m_vehicles,
            i,
            v,
            crossroadReservations,
            m_elapsedTime);

        if (crossroadDecision.shouldRelease) {
            v.reservedCrossroadId = -1;
            v.crossroadEngaged = false;
            v.crossroadReleaseTime = 0.0f;
        }

        if (crossroadDecision.reservedCrossroadId >= 0) {
            v.reservedCrossroadId = crossroadDecision.reservedCrossroadId;
        }

        if (crossroadDecision.shouldEngage) {
            v.crossroadEngaged = true;
            v.crossroadReleaseTime = std::max(v.crossroadReleaseTime, m_config.crossroadClearDelay);
        }

        if (crossroadDecision.sawMovingRightThreat) {
            v.lastMovingRightPriorityTime = m_elapsedTime;
        }

        if (crossroadDecision.isRelevant) {
            if (crossroadDecision.shouldStop && !v.crossroadEngaged) {
                v.crossroadWaitTime += dt;
                v.crossroadClearTime = 0.0f;
            } else {
                v.crossroadClearTime += dt;
                if (crossroadDecision.ownsReservation || v.crossroadEngaged) {
                    v.crossroadWaitTime = 0.0f;
                }
            }
        } else {
            v.crossroadWaitTime = 0.0f;
            v.crossroadClearTime = 0.0f;
            v.crossroadReleaseTime = 0.0f;
            v.lastMovingRightPriorityTime = -1000000.0f;
            if (v.reservedCrossroadId < 0) {
                v.crossroadEngaged = false;
            }
        }

        if (v.crossroadReleaseTime > 0.0f) {
            v.crossroadReleaseTime = std::max(0.0f, v.crossroadReleaseTime - dt);
        }

        float leaderGap = 999999.0f;
        const Vehicle* leader = findLeader(m_vehicles[i], i, leaderGap);

        // GPU path toggle point: per-vehicle intent computation.
        const VehicleIntent intent = vehicleBehavior.computeIntent(
            *this,
            v,
            leader,
            leaderGap,
            crossroadDecision.shouldStop && !v.crossroadEngaged,
            dt);
        v.acceleration = intent.acceleration;

        v.speed = std::max(0.0f, v.speed + v.acceleration * dt);

        const int dir = road.lanes[static_cast<std::size_t>(v.laneId)].direction;
        v.s += static_cast<float>(dir) * v.speed * dt;
        if (v.s >= road.length) {
            if (road.endConnection.has_value()) {
                const auto& c = *road.endConnection;
                if (c.roadId < m_network.roads.size() && c.laneId >= 0 &&
                    static_cast<std::size_t>(c.laneId) < m_network.roads[c.roadId].lanes.size() &&
                    m_network.roads[c.roadId].length > 0.0f) {
                    v.roadId = c.roadId;
                    v.laneId = c.laneId;
                    // Crossing an explicit road connection resets the lane-progress
                    // to the start of the destination lane instead of carrying
                    // residual distance from the source road.
                    v.s = 0.0f;
                } else {
                    v.s = wrapDistance(v.s, road.length);
                }
            } else {
                v.s = wrapDistance(v.s, road.length);
            }
            ++m_wrapCountAccumulator;
        } else if (v.s < 0.0f) {
            if (road.startConnection.has_value()) {
                const auto& c = *road.startConnection;
                if (c.roadId < m_network.roads.size() && c.laneId >= 0 &&
                    static_cast<std::size_t>(c.laneId) < m_network.roads[c.roadId].lanes.size() &&
                    m_network.roads[c.roadId].length > 0.0f) {
                    v.roadId = c.roadId;
                    v.laneId = c.laneId;
                    // Same behavior for start-connection transfers: enter the
                    // destination lane at its canonical start.
                    v.s = 0.0f;
                } else {
                    v.s = wrapDistance(v.s, road.length);
                }
            } else {
                v.s = wrapDistance(v.s, road.length);
            }
            ++m_wrapCountAccumulator;
        }
    }

    // GPU path toggle point: footprint projection / overlap pass.
    // Current behavior keeps CPU and GPU modes numerically consistent until a
    // dedicated compute implementation is introduced.
    if (useGpuComputePath) {
        TrafficPhysics::enforceNoOverlap(
            *this,
            m_vehicles,
            next,
            std::max(m_config.minimumGap, m_config.physicsMinimumGap),
            dt);
    } else {
        TrafficPhysics::enforceNoOverlap(
            *this,
            m_vehicles,
            next,
            std::max(m_config.minimumGap, m_config.physicsMinimumGap),
            dt);
    }

    m_vehicles = std::move(next);

    float speedSum = 0.0f;
    std::size_t queued = 0;
    for (const Vehicle& vehicle : m_vehicles) {
        speedSum += vehicle.speed;
        if (vehicle.speed < 2.0f) ++queued;
    }

    m_stats.averageSpeed = m_vehicles.empty() ? 0.0f : speedSum / static_cast<float>(m_vehicles.size());
    m_stats.averageQueueLength = m_vehicles.empty() ? 0.0f : static_cast<float>(queued);
    m_stats.throughputPerSecond = m_elapsedTime > 0.0f
        ? static_cast<float>(m_wrapCountAccumulator) / m_elapsedTime
        : 0.0f;
}

} // namespace traffic_flow
