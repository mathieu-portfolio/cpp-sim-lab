#include "RoadTopologyServices.hpp"

#include <algorithm>
#include <cmath>

namespace traffic_flow {
namespace {
float wrapDistance(float s, float length) {
    if (length <= 0.0f) return 0.0f;
    float wrapped = std::fmod(s, length);
    if (wrapped < 0.0f) wrapped += length;
    return wrapped;
}

float cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
bool pointsClose(const Vec2& a, const Vec2& b, float radius) { return (a - b).lengthSquared() <= radius * radius; }

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

std::vector<Vec2> buildDrivePoints(const std::vector<Vec2>& rawPoints, float minPointDistance) {
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
    return points[static_cast<std::size_t>(lo)] + (points[static_cast<std::size_t>(hi)] - points[static_cast<std::size_t>(lo)]) * alpha;
}
}

void RoadGeometryCacheBuilder::rebuild(SimulationConfig& config, RoadSegment& road) {
    config.arcLengthSamplesPerSpan = std::clamp(config.arcLengthSamplesPerSpan, 8, 96);
    road.drivePoints = buildDrivePoints(road.controlPoints, config.roadSmoothingMinPointDistance);
    const int spans = std::max(1, static_cast<int>(road.drivePoints.empty() ? road.controlPoints.size() : road.drivePoints.size()));
    const int samples = std::max(2, spans * std::max(2, config.arcLengthSamplesPerSpan));
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
void RoadGeometryCacheBuilder::rebuildAll(SimulationConfig& config, RoadNetwork& network) {
    for (RoadSegment& road : network.roads) rebuild(config, road);
}

void CrossroadDetector::rebuildRoadConnections(RoadNetwork& network) {
    constexpr float connectionSnapRadius = 20.0f;
    for (RoadSegment& road : network.roads) { road.startConnection.reset(); road.endConnection.reset(); }
    for (std::size_t a = 0; a < network.roads.size(); ++a) {
        RoadSegment& roadA = network.roads[a];
        if (roadA.controlPoints.size() < 2 || roadA.lanes.empty()) continue;
        const Vec2 aStart = roadA.controlPoints.front(); const Vec2 aEnd = roadA.controlPoints.back();
        for (std::size_t b = 0; b < network.roads.size(); ++b) {
            if (a == b) continue; const RoadSegment& roadB = network.roads[b];
            if (roadB.controlPoints.size() < 2 || roadB.lanes.empty()) continue;
            if (!roadA.endConnection && pointsClose(aEnd, roadB.controlPoints.front(), connectionSnapRadius)) roadA.endConnection = RoadConnection{b, roadA.lanes.front().direction >= 0 ? 0 : 1};
            if (!roadA.startConnection && pointsClose(aStart, roadB.controlPoints.back(), connectionSnapRadius)) roadA.startConnection = RoadConnection{b, roadA.lanes.front().direction >= 0 ? 0 : 1};
        }
    }
}
void CrossroadDetector::rebuild(const SimulationConfig& config, const Simulation& simulation, RoadNetwork& network) {
    network.crossroads.clear();
    for (std::size_t roadId = 0; roadId < network.roads.size(); ++roadId) {
        const RoadSegment& road = network.roads[roadId];
        if (road.length <= 1.0f || road.arcLengthCache.size() < 8) continue;
        const int samples = std::max(64, static_cast<int>(road.drivePoints.empty() ? road.controlPoints.size() : road.drivePoints.size()) * std::max(12, config.arcLengthSamplesPerSpan));
        struct Sample { Vec2 p; float s; }; std::vector<Sample> centerline; centerline.reserve(static_cast<std::size_t>(samples + 1));
        for (int i = 0; i <= samples; ++i) { float s = road.length * static_cast<float>(i) / static_cast<float>(samples); centerline.push_back({simulation.sampleRoadCenter(roadId, s), s}); }
        for (int a = 0; a < samples; ++a) for (int b = a + 2; b < samples; ++b) {
            if (a == 0 && b == samples - 1) continue; float u=0,v=0; if (!lineIntersection(centerline[a].p, centerline[a+1].p, centerline[b].p, centerline[b+1].p, u, v)) continue;
            float sA = centerline[a].s + (centerline[a+1].s - centerline[a].s) * u; float sB = centerline[b].s + (centerline[b+1].s - centerline[b].s) * v;
            if (std::fabs(sA - sB) < config.crossroadYieldLookahead) continue; if (road.length - std::fabs(sA - sB) < config.crossroadYieldLookahead) continue;
            Vec2 pos = centerline[a].p + (centerline[a+1].p - centerline[a].p) * u; bool merged = false;
            for (Crossroad& existing : network.crossroads) if ((existing.position - pos).lengthSquared() < 28.0f * 28.0f) { existing.approaches.push_back({roadId, wrapDistance(sA, road.length)}); existing.approaches.push_back({roadId, wrapDistance(sB, road.length)}); merged = true; break; }
            if (!merged) { Crossroad c; c.position=pos; c.approaches.push_back({roadId, wrapDistance(sA, road.length)}); c.approaches.push_back({roadId, wrapDistance(sB, road.length)}); network.crossroads.push_back(std::move(c)); }
        }
    }
}

} // namespace traffic_flow
