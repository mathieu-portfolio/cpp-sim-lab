#include "Simulation.hpp"

#include "CrossroadPolicy.hpp"
#include "RoadTopologyServices.hpp"
#include "TrafficLifecycle.hpp"
#include "TrafficPhysics.hpp"
#include "VehicleBehavior.hpp"

#include <algorithm>
#include <cmath>

namespace traffic_flow
{
    namespace
    {
        void normalizeLaneDirections(RoadSegment& road)
        {
            for (auto& lane : road.lanes)
            {
                lane.direction = lane.lateralOffset <= 0.0f ? 1 : -1;
            }
        }

        float wrapDistance(float s, float length)
        {
            if (length <= 0.0f)
                return 0.0f;
            float wrapped = std::fmod(s, length);
            if (wrapped < 0.0f)
                wrapped += length;
            return wrapped;
        }

        Vec2 sampleByT(const RoadSegment &road, float t)
        {
            const std::vector<Vec2> &points = road.drivePoints.empty() ? road.controlPoints : road.drivePoints;
            if (points.empty())
                return {};
            if (points.size() == 1)
                return points.front();
            const float clampedT = std::clamp(t, 0.0f, 1.0f);
            const float scaled = clampedT * static_cast<float>(points.size() - 1);
            const int lo = static_cast<int>(std::floor(scaled));
            const int hi = std::min(static_cast<int>(points.size() - 1), lo + 1);
            const float alpha = scaled - static_cast<float>(lo);
            return points[static_cast<std::size_t>(lo)] + (points[static_cast<std::size_t>(hi)] - points[static_cast<std::size_t>(lo)]) * alpha;
        }
    }

    struct Simulation::TrafficStateData
    {
        TrafficState state;
    };

    Simulation::Simulation(SimulationConfig config) : m_config(config), m_stateData(new TrafficStateData{})
    {
        sanitizeConfig();
        resetDefaultRoad();
        generateTraffic();
    }

    void Simulation::sanitizeConfig()
    {
        m_config.vehicleCount = std::clamp<std::size_t>(m_config.vehicleCount, 0, 500);
        m_config.desiredSpeed = std::clamp(m_config.desiredSpeed, 1.0f, 60.0f);
        m_config.maxAcceleration = std::clamp(m_config.maxAcceleration, 0.0f, 4.0f);
        m_config.comfortableBraking = std::clamp(m_config.comfortableBraking, 0.1f, 5.0f);
        m_config.minimumGap = std::clamp(m_config.minimumGap, 0.0f, 20.0f);
        m_config.desiredTimeHeadway = std::clamp(m_config.desiredTimeHeadway, 0.1f, 4.0f);
        m_config.laneWidth = std::clamp(m_config.laneWidth, 4.0f, 80.0f);
    }

    void Simulation::resetDefaultRoad()
    {
        RoadSegment road;
        road.controlPoints = {{120, 350}, {980, 350}};
        road.lanes = {{1, -m_config.laneWidth * 0.5f}, {-1, m_config.laneWidth * 0.5f}};
        normalizeLaneDirections(road);
        RoadGeometryCacheBuilder::rebuild(m_config, road);
        m_network.roads = {road};
        CrossroadDetector::rebuild(m_config, *this, m_network);
    }

    void Simulation::rebuildRoadCaches()
    {
        for (RoadSegment& road : m_network.roads)
            normalizeLaneDirections(road);
        RoadGeometryCacheBuilder::rebuildAll(m_config, m_network);
        CrossroadDetector::rebuildRoadConnections(m_network);
        CrossroadDetector::rebuild(m_config, *this, m_network);
    }

    void Simulation::reset()
    {
        resetDefaultRoad();
        generateTraffic();
    }

    void Simulation::clearTraffic()
    {
        TrafficResetService::clear(m_vehicles, m_stats, m_stateData->state);
    }

    void Simulation::generateTraffic()
    {
        sanitizeConfig();
        clearTraffic();
        TrafficSpawner::generate(*this, m_vehicles);
    }

    void Simulation::notifyRoadsEdited()
    {
        rebuildRoadCaches();
        clearTraffic();
    }

    Vec2 Simulation::sampleRoadCenter(std::size_t roadId, float s) const
    {
        const RoadSegment &road = m_network.roads[roadId];
        if (road.length <= 0 || road.arcLengthCache.size() < 2)
            return sampleByT(road, 0);
        float clampedS = wrapDistance(s, road.length);
        auto it = std::lower_bound(road.arcLengthCache.begin(), road.arcLengthCache.end(), clampedS);
        std::size_t hi = static_cast<std::size_t>(std::distance(road.arcLengthCache.begin(), it));
        if (hi == 0)
            return sampleByT(road, 0);
        std::size_t lo = hi - 1;
        float s0 = road.arcLengthCache[lo], s1 = road.arcLengthCache[hi];
        float a = (s1 > s0) ? (clampedS - s0) / (s1 - s0) : 0;
        float t0 = float(lo) / float(road.arcLengthCache.size() - 1), t1 = float(hi) / float(road.arcLengthCache.size() - 1);
        return sampleByT(road, t0 + (t1 - t0) * a);
    }

    Vec2 Simulation::sampleRoadTangent(std::size_t roadId, int laneId, float s) const
    {
        const RoadSegment &road = m_network.roads[roadId];
        if (laneId < 0 || static_cast<std::size_t>(laneId) >= road.lanes.size())
            return {1, 0};
        Vec2 t = sampleRoadCenter(roadId, s + 0.5f) - sampleRoadCenter(roadId, s - 0.5f);
        t = (t.lengthSquared() < 1e-6f) ? Vec2{1, 0} : t.normalized();
        if (road.lanes[static_cast<std::size_t>(laneId)].direction < 0)
            t = t * -1.0f;
        return t;
    }

    Vec2 Simulation::sampleLanePosition(std::size_t roadId, int laneId, float s) const
    {
        const RoadSegment &road = m_network.roads[roadId];
        if (laneId < 0 || static_cast<std::size_t>(laneId) >= road.lanes.size())
            return sampleRoadCenter(roadId, s);
        Vec2 c = sampleRoadCenter(roadId, s);
        Vec2 t = sampleRoadCenter(roadId, s + 0.5f) - sampleRoadCenter(roadId, s - 0.5f);
        t = (t.lengthSquared() < 1e-6f) ? Vec2{1, 0} : t.normalized();
        return c + Vec2{-t.y, t.x} * road.lanes[static_cast<std::size_t>(laneId)].lateralOffset;
    }

    float Simulation::distanceToCrossroadAlongLane(const Vehicle &v, float cs) const
    {
        const RoadSegment &r = m_network.roads[v.roadId];
        int d = r.lanes[static_cast<std::size_t>(v.laneId)].direction;
        return d >= 0 ? wrapDistance(cs - v.s, r.length) : wrapDistance(v.s - cs, r.length);
    }

    float Simulation::distanceAheadOnLane(const Vehicle &a, const Vehicle &b) const
    {
        if (a.roadId != b.roadId || a.laneId != b.laneId)
            return 999999.0f;
        const RoadSegment &r = m_network.roads[a.roadId];
        int d = r.lanes[static_cast<std::size_t>(a.laneId)].direction;
        return d >= 0 ? wrapDistance(b.s - a.s, r.length) : wrapDistance(a.s - b.s, r.length);
    }

    const Vehicle *Simulation::findLeader(const Vehicle &vehicle, std::size_t idx, float &outGap) const
    {
        const Vehicle *leader = nullptr;
        outGap = 999999.0f;
        for (std::size_t j = 0; j < m_vehicles.size(); ++j)
        {
            if (j == idx)
                continue;
            float dist = distanceAheadOnLane(vehicle, m_vehicles[j]);
            if (dist <= 0.01f || dist >= outGap)
                continue;
            float gap = dist - (vehicle.length + m_vehicles[j].length) * 0.5f;
            if (gap < outGap)
            {
                outGap = gap;
                leader = &m_vehicles[j];
            }
        }
        return leader;
    }

    bool Simulation::findNearestCrossroadAhead(const Vehicle &v, std::size_t &ci, float &as, float &d) const
    {
        ci = static_cast<std::size_t>(-1);
        as = 0;
        d = 999999;
        for (std::size_t i = 0; i < m_network.crossroads.size(); ++i)
            for (const auto &a : m_network.crossroads[i].approaches)
                if (a.roadId == v.roadId)
                {
                    float dd = distanceToCrossroadAlongLane(v, a.s);
                    if (dd < d)
                    {
                        d = dd;
                        as = a.s;
                        ci = i;
                    }
                }
        return ci != static_cast<std::size_t>(-1);
    }

    bool Simulation::isVehicleInsideCrossroad(const Vehicle &v, std::size_t ci) const
    {
        if (ci >= m_network.crossroads.size())
            return false;
        const RoadSegment &r = m_network.roads[v.roadId];
        for (const auto &a : m_network.crossroads[ci].approaches)
            if (a.roadId == v.roadId)
            {
                float d = distanceToCrossroadAlongLane(v, a.s);
                if (d <= m_config.crossroadStopRadius || d >= r.length - m_config.crossroadStopRadius)
                    return true;
            }
        return false;
    }

    bool Simulation::isInsideCrossroadSpawnClearance(std::size_t, float) const { return false; }

    void Simulation::runIntentPhase(float dt, std::vector<Vehicle> &next)
    {
        RightPriorityWithDeadlockBreakerPolicy p;
        DefaultVehicleBehavior b;
        auto reservations = p.rebuildReservations(*this, m_vehicles);
        for (std::size_t i = 0; i < m_vehicles.size(); ++i)
        {
            auto dec = p.decide(*this, m_vehicles, i, next[i], reservations, m_stateData->state.elapsedTime);
            float gap = 999999;
            const Vehicle *leader = findLeader(m_vehicles[i], i, gap);
            auto intent = b.computeIntent(*this, next[i], leader, gap, dec.shouldStop && !next[i].crossroadEngaged, dt);
            next[i].acceleration = intent.acceleration;
        }
    }

    void Simulation::runIntegratePhase(float dt, std::vector<Vehicle> &next)
    {
        for (auto &v : next)
            v.speed = std::max(0.0f, v.speed + v.acceleration * dt);
    }

    void Simulation::runConnectionPhase(std::vector<Vehicle> &next)
    {
        for (auto &v : next)
        {
            if (v.roadId >= m_network.roads.size())
                continue;
            const RoadSegment &r = m_network.roads[v.roadId];
            int dir = r.lanes[static_cast<std::size_t>(v.laneId)].direction;
            v.s += dir * v.speed * 0.016f;
            if (v.s > r.length)
            {
                v.s = r.length;
                ++m_stateData->state.wrapCountAccumulator;
            }
            if (v.s < 0)
            {
                v.s = 0;
                ++m_stateData->state.wrapCountAccumulator;
            }
        }
    }

    void Simulation::runPhysicsPhase(float dt, std::vector<Vehicle> &next, bool)
    {
        TrafficPhysics::enforceNoOverlap(*this, m_vehicles, next, std::max(m_config.minimumGap, m_config.physicsMinimumGap), dt);
    }

    void Simulation::runStatsPhase()
    {
        float sum = 0;
        std::size_t queued = 0;
        for (const auto &v : m_vehicles)
        {
            sum += v.speed;
            if (v.speed < 2)
                ++queued;
        }
        m_stats.averageSpeed = m_vehicles.empty() ? 0 : sum / static_cast<float>(m_vehicles.size());
        m_stats.averageQueueLength = static_cast<float>(queued);
        m_stats.throughputPerSecond = m_stateData->state.elapsedTime > 0 ? static_cast<float>(m_stateData->state.wrapCountAccumulator) / m_stateData->state.elapsedTime : 0;
    }

    void Simulation::update(float dt)
    {
        sanitizeConfig();
        m_stateData->state.elapsedTime += dt;
        std::vector<Vehicle> next = m_vehicles;
        runIntentPhase(dt, next);
        runIntegratePhase(dt, next);
        runConnectionPhase(next);
        runPhysicsPhase(dt, next, m_config.execution.computeBackend == simfw::simulation::ComputeBackend::GpuCompute);
        m_vehicles = std::move(next);
        runStatsPhase();
    }

} // namespace traffic_flow
