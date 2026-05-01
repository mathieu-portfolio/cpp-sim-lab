#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <random/Random.hpp>

#include <utility>

namespace traffic_flow_cpu {
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

Vec2 sampleByT(const RoadSegment& road, float t) {
    if (road.controlPoints.empty()) return {};
    if (road.controlPoints.size() == 1) return road.controlPoints.front();

    const int pointCount = static_cast<int>(road.controlPoints.size());
    const int spans = pointCount;
    const float wrappedT = t - std::floor(t);
    const float scaled = wrappedT * static_cast<float>(spans);
    const int seg = std::min(spans - 1, static_cast<int>(scaled));
    const float lt = scaled - static_cast<float>(seg);
    const int i0 = wrapIndex(seg - 1, pointCount);
    const int i1 = wrapIndex(seg, pointCount);
    const int i2 = wrapIndex(seg + 1, pointCount);
    const int i3 = wrapIndex(seg + 2, pointCount);
    return catmullRom(road.controlPoints[i0], road.controlPoints[i1], road.controlPoints[i2], road.controlPoints[i3], lt);
}

} // namespace

Simulation::Simulation(SimulationConfig config) : m_config(config) {
    resetDefaultRoad();
    clearTraffic();
}

void Simulation::resetDefaultRoad() {
    RoadSegment road;
    road.controlPoints = {{80.0f, 350.0f}, {300.0f, 280.0f}, {600.0f, 420.0f}, {1000.0f, 350.0f}};
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
    const int spans = std::max(1, static_cast<int>(road.controlPoints.size()));
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

        const int samples = std::max(32, static_cast<int>(road.controlPoints.size()) * std::max(12, m_config.arcLengthSamplesPerSpan));
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
    clearTraffic();
}

void Simulation::clearTraffic() {
    m_vehicles.clear();
    m_throughputAccumulator = 0.0f;
    m_wrapCountAccumulator = 0;
    m_queueAccumulator = 0.0f;
    m_queueSamples = 0.0f;
    m_stats = {};
}

void Simulation::notifyRoadsEdited() {
    rebuildRoadCaches();
    clearTraffic();
}

void Simulation::generateTraffic() {
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
    std::vector<std::size_t> perLaneCount(laneCount, 0);
    for (std::size_t i = 0; i < m_config.vehicleCount; ++i) {
        ++perLaneCount[i % laneCount];
    }

    for (std::size_t laneSlot = 0; laneSlot < laneCount; ++laneSlot) {
        const auto& laneRef = spawnLanes[laneSlot];
        const RoadSegment& road = m_network.roads[laneRef.first];
        const std::size_t count = perLaneCount[laneSlot];
        if (count == 0 || road.length <= 1.0f) continue;

        for (std::size_t i = 0; i < count; ++i) {
            Vehicle v;
            v.roadId = laneRef.first;
            v.laneId = laneRef.second;
            v.s = (static_cast<float>(i) + 0.5f) / static_cast<float>(count) * road.length;
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

bool Simulation::hasRightSideThreatAtCrossroad(const Vehicle& vehicle, std::size_t vehicleIndex) const {
    for (const Crossroad& crossroad : m_network.crossroads) {
        float ownApproachS = 0.0f;
        float ownDistance = 999999.0f;
        bool approaching = false;
        for (const CrossroadApproach& approach : crossroad.approaches) {
            if (approach.roadId != vehicle.roadId) continue;
            const float distance = distanceToCrossroadAlongLane(vehicle, approach.s);
            if (distance < ownDistance) {
                ownDistance = distance;
                ownApproachS = approach.s;
                approaching = true;
            }
        }

        if (!approaching || ownDistance > m_config.crossroadYieldLookahead) continue;

        const Vec2 ownForward = sampleRoadTangent(vehicle.roadId, vehicle.laneId, ownApproachS);
        const Vec2 ownRight{ownForward.y, -ownForward.x};

        for (std::size_t otherIndex = 0; otherIndex < m_vehicles.size(); ++otherIndex) {
            if (otherIndex == vehicleIndex) continue;
            const Vehicle& other = m_vehicles[otherIndex];

            float otherApproachS = 0.0f;
            float otherDistance = 999999.0f;
            bool otherApproaching = false;
            for (const CrossroadApproach& approach : crossroad.approaches) {
                if (approach.roadId != other.roadId) continue;
                const float distance = distanceToCrossroadAlongLane(other, approach.s);
                if (distance < otherDistance) {
                    otherDistance = distance;
                    otherApproachS = approach.s;
                    otherApproaching = true;
                }
            }

            if (!otherApproaching || otherDistance > m_config.crossroadYieldLookahead) continue;

            if (otherDistance < m_config.crossroadStopRadius * 0.35f &&
                other.speed > vehicle.speed + 0.5f) {
                continue;
            }

            const Vec2 otherForward = sampleRoadTangent(other.roadId, other.laneId, otherApproachS);
            const float rightness = dot(ownRight, otherForward);
            const float parallel = std::fabs(dot(ownForward, otherForward));
            if (rightness > 0.20f && parallel < 0.92f) {
                return true;
            }
        }
    }

    return false;
}

void Simulation::update(float dt) {
    std::vector<Vehicle> next = m_vehicles;
    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        Vehicle& v = next[i];
        const RoadSegment& road = m_network.roads[v.roadId];
        const int dir = road.lanes[static_cast<std::size_t>(v.laneId)].direction;

        bool atYieldZone = false;
        for (const Crossroad& crossroad : m_network.crossroads) {
            for (const CrossroadApproach& approach : crossroad.approaches) {
                if (approach.roadId != v.roadId) continue;
                const float distance = distanceToCrossroadAlongLane(v, approach.s);
                if (distance <= m_config.crossroadYieldLookahead) {
                    atYieldZone = true;
                    break;
                }
            }
            if (atYieldZone) break;
        }

        const bool rightSideThreat = hasRightSideThreatAtCrossroad(m_vehicles[i], i);
        bool shouldStop = false;

        if (atYieldZone && rightSideThreat && v.crossroadReleaseTime <= 0.0f) {
            v.crossroadWaitTime += dt;
            v.crossroadClearTime = 0.0f;
            shouldStop = v.crossroadWaitTime < m_config.crossroadMaxWait;
        } else if (atYieldZone && v.crossroadWaitTime > 0.0f && v.crossroadReleaseTime <= 0.0f) {
            v.crossroadClearTime += dt;
            if (v.crossroadClearTime < m_config.crossroadClearDelay) {
                shouldStop = true;
            } else {
                v.crossroadReleaseTime = 1.0f;
                v.crossroadWaitTime = 0.0f;
                v.crossroadClearTime = 0.0f;
            }
        } else if (!atYieldZone) {
            v.crossroadWaitTime = 0.0f;
            v.crossroadClearTime = 0.0f;
            v.crossroadReleaseTime = 0.0f;
        }

        if (v.crossroadReleaseTime > 0.0f) {
            v.crossroadReleaseTime = std::max(0.0f, v.crossroadReleaseTime - dt);
        }

        if (shouldStop) {
            v.acceleration = -std::max(m_config.comfortableBraking, v.speed / std::max(0.05f, dt));
        } else {
            v.acceleration = idmAcceleration(v, nullptr, 9999.0f);
        }

        v.speed = std::max(0.0f, v.speed + v.acceleration * dt);
        v.s += static_cast<float>(dir) * v.speed * dt;
        if (v.s >= road.length) {
            if (road.endConnection.has_value()) {
                v.roadId = road.endConnection->roadId;
                v.laneId = road.endConnection->laneId;
                v.s = wrapDistance(v.s - road.length, m_network.roads[v.roadId].length);
            } else {
                v.s = wrapDistance(v.s, road.length);
            }
            ++m_wrapCountAccumulator;
        } else if (v.s < 0.0f) {
            if (road.startConnection.has_value()) {
                const auto& c = *road.startConnection;
                v.roadId = c.roadId;
                v.laneId = c.laneId;
                v.s = wrapDistance(m_network.roads[c.roadId].length + v.s, m_network.roads[c.roadId].length);
            } else {
                v.s = wrapDistance(v.s, road.length);
            }
            ++m_wrapCountAccumulator;
        }
    }
    m_vehicles = std::move(next);
}


} // namespace traffic_flow_cpu
