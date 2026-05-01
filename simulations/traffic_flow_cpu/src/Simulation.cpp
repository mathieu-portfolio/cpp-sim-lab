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
}

void Simulation::rebuildRoadCaches() {
    for (RoadSegment& road : m_network.roads) {
        rebuildRoadCache(road);
    }
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

Vec2 Simulation::sampleLanePosition(std::size_t roadId, int laneId, float s) const {
    const RoadSegment& road = m_network.roads[roadId];
    const Vec2 c = sampleRoadCenter(roadId, s);
    const float eps = 0.2f;
    const Vec2 p0 = sampleRoadCenter(roadId, s - eps);
    const Vec2 p1 = sampleRoadCenter(roadId, s + eps);
    Vec2 t = p1 - p0;
    if (t.lengthSquared() < 1e-6f) t = {1.0f, 0.0f};
    else t = t.normalized();
    const Vec2 n{-t.y, t.x};
    return c + n * road.lanes[static_cast<std::size_t>(laneId)].lateralOffset;
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
    float totalLaneLength = 0.0f;
    for (std::size_t roadId = 0; roadId < m_network.roads.size(); ++roadId) {
        const RoadSegment& road = m_network.roads[roadId];
        if (road.length <= 1.0f || road.lanes.empty()) continue;
        for (std::size_t laneId = 0; laneId < road.lanes.size(); ++laneId) {
            spawnLanes.emplace_back(roadId, static_cast<int>(laneId));
            totalLaneLength += road.length;
        }
    }

    if (spawnLanes.empty() || totalLaneLength <= 0.0f) return;

    for (std::size_t i = 0; i < m_config.vehicleCount; ++i) {
        const float target = (static_cast<float>(i) + 0.5f) /
                             std::max(1.0f, static_cast<float>(m_config.vehicleCount)) * totalLaneLength;
        float cursor = 0.0f;
        std::size_t roadId = spawnLanes.back().first;
        int laneId = spawnLanes.back().second;
        float spawnS = 0.0f;

        for (const auto& laneRef : spawnLanes) {
            const RoadSegment& road = m_network.roads[laneRef.first];
            if (target <= cursor + road.length) {
                roadId = laneRef.first;
                laneId = laneRef.second;
                spawnS = target - cursor;
                break;
            }
            cursor += road.length;
        }

        const RoadSegment& road = m_network.roads[roadId];
        Vehicle v;
        v.roadId = roadId;
        v.laneId = laneId;
        v.s = std::clamp(spawnS, 0.0f, road.length);
        v.speed = Random::range(m_config.spawnSpeedMin, m_config.spawnSpeedMax);
        m_vehicles.push_back(v);
    }
}

void Simulation::update(float dt) {
    std::vector<Vehicle> next = m_vehicles;
    for (std::size_t i = 0; i < m_vehicles.size(); ++i) {
        Vehicle& v = next[i];
        const RoadSegment& road = m_network.roads[v.roadId];
        const int dir = road.lanes[static_cast<std::size_t>(v.laneId)].direction;
        v.acceleration = idmAcceleration(v, nullptr, 9999.0f);
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
