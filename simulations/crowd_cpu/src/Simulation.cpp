#include "Simulation.hpp"

#include "thread_pool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random/Random.hpp>
#include <simulation/ParallelUpdate.hpp>
#include <simulation/SpatialQuery.hpp>
#include <simulation/StatsReduction.hpp>
#include <simulation/EntityBrush.hpp>
#include <thread>

namespace crowd_cpu {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;
Vec2 agentPosition(const Agent& a) { return a.position; }
Vec2 obstaclePosition(const Obstacle& o) { return o.position; }
std::size_t workers(){ unsigned int n=std::thread::hardware_concurrency(); return n==0?1:static_cast<std::size_t>(n); }
void normalizeConfigCounts(SimulationConfig& config) {
    constexpr std::size_t defaultCount = SimulationConfig::DefaultAgentCount;

    if (config.agentCount == defaultCount && config.entityCount != defaultCount) {
        config.agentCount = config.entityCount;
    } else {
        config.entityCount = config.agentCount;
    }
}

void resolveObstacleOverlap(
    Agent& agent,
    const std::vector<Obstacle>& obstacles,
    std::span<const std::size_t> candidates,
    SimulationStats& stats
) {
    constexpr float epsilon = 0.0001f;

    for (std::size_t obstacleIndex : candidates) {
        ++stats.obstacleOverlapChecks;

        const Obstacle& obstacle = obstacles[obstacleIndex];
        Vec2 away = agent.position - obstacle.position;
        const float distance = away.length();
        const float minDistance = obstacle.radius + agent.radius;

        if (distance <= epsilon || distance >= minDistance) {
            continue;
        }

        const Vec2 normal = away * (1.0f / distance);
        agent.position = obstacle.position + normal * minDistance;

        const float velocityAlongNormal = Vec2::dot(agent.velocity, normal);

        if (velocityAlongNormal < 0.0f) {
            agent.velocity -= normal * velocityAlongNormal;
        }
    }
}

}

Simulation::Simulation(SimulationConfig config)
    : Base(config), m_agentGrid(config.gridCellSize), m_obstacleGrid(config.gridCellSize),
      m_goal{config.width * 0.85f, config.height * 0.5f}, m_threadPool(std::make_unique<ThreadPool>(workers())),
      m_behaviors(makeDefaultBehaviors()) { reset(); }
Simulation::~Simulation() = default;
Simulation::Simulation(Simulation&&) noexcept = default;
Simulation& Simulation::operator=(Simulation&&) noexcept = default;

void Simulation::beginFrame(){ normalizeConfigCounts(m_config); m_stats={}; m_stats.agentCount=m_entities.size(); m_stats.entityCount=m_entities.size(); m_stats.obstacleCount=m_obstacles.size(); m_config.entityCount=m_entities.size(); }
void Simulation::setGoal(Vec2 g){ m_goal = g; }
void Simulation::addObstacle(Vec2 p){ m_obstacles.push_back({p, m_config.obstacleRadius}); }
void Simulation::addObstacle(Vec2 p, float radius){ m_obstacles.push_back({p, radius}); }
void Simulation::clearObstacles(){ m_obstacles.clear(); }
void Simulation::spawn(const Vec2& position){
    simfw::simulation::spawnBrush(m_entities.size(), m_config.maxAgentCount, m_config.spawnCount, [this, position](){
        Vec2 spawnPos = position + simfw::simulation::randomDiscOffset(m_config.brushRadius);
        spawnPos.x = std::clamp(spawnPos.x, 0.0f, m_config.width);
        spawnPos.y = std::clamp(spawnPos.y, 0.0f, m_config.height);
        m_entities.push_back({spawnPos, Vec2{}, m_config.agentRadius});
    });
}


void Simulation::reset() {
    normalizeConfigCounts(m_config);
    m_entities.clear(); m_entities.reserve(m_config.agentCount);
    m_previousAgents.clear(); m_previousAgents.reserve(m_config.agentCount);
    for (std::size_t i=0;i<m_config.agentCount;++i){
        m_entities.push_back({Vec2{Random::range(0.0f, m_config.width*0.3f), Random::range(0.0f,m_config.height)}, Vec2{}, m_config.agentRadius});
    }
}

void Simulation::buildSpatialIndexes() {
    m_agentGrid.setCellSize(m_config.gridCellSize); m_obstacleGrid.setCellSize(m_config.gridCellSize);
    if (m_config.execution.useSpatialGrid) {
        m_agentGrid.build(m_previousAgents, agentPosition); m_obstacleGrid.build(m_obstacles, obstaclePosition);
        m_stats.occupiedGridCells = m_agentGrid.getCells().size(); m_stats.occupiedObstacleGridCells = m_obstacleGrid.getCells().size();
    } else { m_agentGrid.clear(); m_obstacleGrid.clear(); }
}

void Simulation::buildFlowField() {
    m_gridWidth = static_cast<std::size_t>(m_config.width / m_config.gridCellSize) + 1;
    m_gridHeight = static_cast<std::size_t>(m_config.height / m_config.gridCellSize) + 1;
    const std::size_t count = m_gridWidth * m_gridHeight;
    m_costField.assign(count, 1.0f);
    m_integrationField.assign(count, std::numeric_limits<float>::infinity());
    for (std::size_t y=0; y<m_gridHeight; ++y) for (std::size_t x=0; x<m_gridWidth; ++x) {
        Vec2 c{(x+0.5f)*m_config.gridCellSize,(y+0.5f)*m_config.gridCellSize};
        for (const Obstacle& o: m_obstacles) if ((c-o.position).length() < o.radius) { m_costField[y*m_gridWidth+x]=255.0f; break; }
    }
    auto goalX = std::clamp(static_cast<int>(m_goal.x/m_config.gridCellSize),0,static_cast<int>(m_gridWidth-1));
    auto goalY = std::clamp(static_cast<int>(m_goal.y/m_config.gridCellSize),0,static_cast<int>(m_gridHeight-1));
    std::queue<std::size_t> q; std::size_t gi=static_cast<std::size_t>(goalY)*m_gridWidth+static_cast<std::size_t>(goalX);
    m_integrationField[gi]=0.0f; q.push(gi);
    constexpr int dx[4]={1,-1,0,0}; constexpr int dy[4]={0,0,1,-1};
    while(!q.empty()){
        std::size_t i=q.front(); q.pop(); std::size_t x=i%m_gridWidth; std::size_t y=i/m_gridWidth;
        for(int k=0;k<4;++k){ int nx=static_cast<int>(x)+dx[k], ny=static_cast<int>(y)+dy[k]; if(nx<0||ny<0||nx>=static_cast<int>(m_gridWidth)||ny>=static_cast<int>(m_gridHeight)) continue;
            std::size_t ni=static_cast<std::size_t>(ny)*m_gridWidth+static_cast<std::size_t>(nx); if(m_costField[ni]>=255.0f) continue;
            float nd = m_integrationField[i] + m_costField[ni]; if(nd < m_integrationField[ni]) { m_integrationField[ni]=nd; q.push(ni);} }
    }
}

Vec2 Simulation::sampleFlow(Vec2 worldPos) const {
    if (m_integrationField.empty() || m_gridWidth == 0 || m_gridHeight == 0) return Vec2{};
    const float cellSize = m_config.gridCellSize;
    auto sample = [&](Vec2 p) {
        const float gx = std::clamp(p.x / cellSize, 0.0f, static_cast<float>(m_gridWidth - 1));
        const float gy = std::clamp(p.y / cellSize, 0.0f, static_cast<float>(m_gridHeight - 1));
        const std::size_t x0 = static_cast<std::size_t>(gx);
        const std::size_t y0 = static_cast<std::size_t>(gy);
        const std::size_t x1 = std::min(x0 + 1, m_gridWidth - 1);
        const std::size_t y1 = std::min(y0 + 1, m_gridHeight - 1);
        const float tx = gx - static_cast<float>(x0);
        const float ty = gy - static_cast<float>(y0);
        const float i00 = m_integrationField[y0 * m_gridWidth + x0];
        const float i10 = m_integrationField[y0 * m_gridWidth + x1];
        const float i01 = m_integrationField[y1 * m_gridWidth + x0];
        const float i11 = m_integrationField[y1 * m_gridWidth + x1];
        if (!std::isfinite(i00) || !std::isfinite(i10) || !std::isfinite(i01) || !std::isfinite(i11)) {
            return std::numeric_limits<float>::infinity();
        }
        const float top = i00 + (i10 - i00) * tx;
        const float bottom = i01 + (i11 - i01) * tx;
        return top + (bottom - top) * ty;
    };
    const float h = cellSize;
    const float fxm = sample(Vec2{worldPos.x - h, worldPos.y});
    const float fxp = sample(Vec2{worldPos.x + h, worldPos.y});
    const float fym = sample(Vec2{worldPos.x, worldPos.y - h});
    const float fyp = sample(Vec2{worldPos.x, worldPos.y + h});
    if (!std::isfinite(fxm) || !std::isfinite(fxp) || !std::isfinite(fym) || !std::isfinite(fyp)) return Vec2{};
    const float inv2h = 0.5f / h;
    return Vec2{-(fxp - fxm) * inv2h, -(fyp - fym) * inv2h};
}

void Simulation::updateAgents(float dt) {
    const float obstacleQueryRadius = m_config.obstacleAvoidanceRadius + m_config.obstacleRadius;
    auto worker = [&](std::size_t begin, std::size_t end, Scratch& s, SimulationStats& st){
        for(std::size_t i=begin;i<end;++i){
            simfw::simulation::collectCandidates(
                m_agentGrid,
                m_previousAgents[i].position,
                m_config.separationRadius,
                simfw::simulation::makeSpatialQueryOptionsExcluding(
                    m_config.execution.useSpatialGrid,
                    m_previousAgents.size(),
                    i
                ),
                s.candidates
            );
            st.neighborCandidates += s.candidates.size();
            simfw::simulation::collectCandidates(m_obstacleGrid, m_previousAgents[i].position, obstacleQueryRadius,
                simfw::simulation::makeSpatialQueryOptions(m_config.execution.useSpatialGrid,m_obstacles.size()), s.secondaryNeighbors);
            st.obstacleCandidates += s.secondaryNeighbors.size();
            BehaviorContext ctx{m_config,m_previousAgents,m_obstacles,m_integrationField,m_gridWidth,m_gridHeight,{s.candidates,s.secondaryNeighbors},st};
            Vec2 acc = computeAcceleration(i, ctx, m_behaviors);
            Agent& a=m_entities[i]; const Agent& p=m_previousAgents[i];
            a.velocity = limitLength(p.velocity + acc*dt, m_config.maxSpeed);
            a.position = p.position + a.velocity*dt;
            resolveObstacleOverlap(a, m_obstacles, s.secondaryNeighbors, st);
            a.position.x = std::clamp(a.position.x, 0.0f, m_config.width); a.position.y = std::clamp(a.position.y, 0.0f, m_config.height);
            if ((a.position - m_goal).length() <= m_config.goalRadius) ++st.reachedGoalCount;
        }
    };
    simfw::runParallelUpdate<ThreadPool, Scratch, SimulationStats>(
        m_threadPool.get(),
        m_entities.size(),
        MinItemsPerParallelTask,
        m_config.execution.useParallelUpdate,
        worker,
        [this](const SimulationStats& s) {
            simfw::sumStatsMembers(
                m_stats,
                s,
                &SimulationStats::neighborCandidates,
                &SimulationStats::neighborChecks,
                &SimulationStats::obstacleCandidates,
                &SimulationStats::obstacleChecks,
                &SimulationStats::obstacleOverlapChecks,
                &SimulationStats::reachedGoalCount
            );
        }
    );
}

void Simulation::update(float dt) {
    beginFrame(); m_previousAgents = m_entities; buildSpatialIndexes(); buildFlowField(); updateAgents(dt);
}

} // namespace crowd_cpu
