#include "Simulation.hpp"

#include "thread_pool.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <random/Random.hpp>
#include <simulation/EntityBrush.hpp>
#include <simulation/ObstaclePhysics.hpp>
#include <simulation/ParallelUpdate.hpp>
#include <simulation/SimulationUtils.hpp>
#include <simulation/SpatialQuery.hpp>
#include <simulation/StatsReduction.hpp>

namespace crowd_cpu {
namespace {
constexpr std::size_t MinItemsPerParallelTask = 256;
Vec2 agentPosition(const Agent &a) { return a.position; }
void normalizeConfigCounts(SimulationConfig &config) {
  simfw::simulation::syncEntityCount(SimulationConfig::DefaultAgentCount,
                                     config.agentCount, config);
}
} // namespace

Simulation::Simulation(SimulationConfig config)
    : Base(config), m_agentGrid(config.gridCellSize),
      m_goal{config.width * 0.85f, config.height * 0.5f},
      m_threadPool(std::make_unique<ThreadPool>(
          simfw::simulation::hardwareWorkerCount())),
      m_behaviors(makeDefaultBehaviors()) {
  reset();
}
Simulation::~Simulation() = default;
Simulation::Simulation(Simulation &&) noexcept = default;
Simulation &Simulation::operator=(Simulation &&) noexcept = default;

void Simulation::beginFrame() {
  normalizeConfigCounts(m_config);
  m_stats = {};
  m_stats.agentCount = m_entities.size();
  m_stats.entityCount = m_entities.size();
  m_config.entityCount = m_entities.size();
}
void Simulation::setGoal(Vec2 g) { m_goal = g; }
void Simulation::spawn(const Vec2 &position) {
  simfw::simulation::spawnBrush(
      m_entities.size(), m_config.maxAgentCount, m_config.spawnCount,
      [this, position]() {
        Vec2 spawnPos = position + simfw::simulation::randomDiscOffset(
                                       m_config.brushRadius);
        spawnPos.x = std::clamp(spawnPos.x, 0.0f, m_config.width);
        spawnPos.y = std::clamp(spawnPos.y, 0.0f, m_config.height);
        m_entities.push_back({spawnPos, Vec2{}, m_config.agentRadius});
      });
}

void Simulation::reset() {
  normalizeConfigCounts(m_config);
  m_entities.clear();
  m_entities.reserve(m_config.agentCount);
  m_previousAgents.clear();
  m_previousAgents.reserve(m_config.agentCount);
  m_obstacleMask.resize(static_cast<int>(m_config.width),
                        static_cast<int>(m_config.height));
  for (std::size_t i = 0; i < m_config.agentCount; ++i) {
    m_entities.push_back({Vec2{Random::range(0.0f, m_config.width * 0.3f),
                               Random::range(0.0f, m_config.height)},
                          Vec2{}, m_config.agentRadius});
  }
  m_activeScenario.reset();
  m_scenarioTime = 0.0f;
}

void Simulation::loadScenario(CanonicalScenario scenario) {
  normalizeConfigCounts(m_config);
  m_entities.clear();
  m_entities.reserve(m_config.agentCount);
  m_obstacleMask.resize(static_cast<int>(m_config.width),
                        static_cast<int>(m_config.height));
  m_obstacleMask.clear();

  auto spawnAgent = [this](float minX, float maxX, float minY, float maxY) {
    m_entities.push_back(
        {Vec2{Random::range(minX, maxX), Random::range(minY, maxY)}, Vec2{},
         m_config.agentRadius});
  };

  const float width = m_config.width;
  const float height = m_config.height;
  const std::size_t count = m_config.agentCount;

  switch (scenario) {
  case CanonicalScenario::CorridorBidirectionalFlow: {
    m_goal = Vec2{width * 0.9f, height * 0.5f};
    m_obstacleMask.paintCircle(Vec2{width * 0.5f, height * 0.15f},
                               width * 0.32f,
                               simfw::simulation::ObstaclePaintMode::Block);
    m_obstacleMask.paintCircle(Vec2{width * 0.5f, height * 0.85f},
                               width * 0.32f,
                               simfw::simulation::ObstaclePaintMode::Block);

    const std::size_t halfCount = count / 2;
    for (std::size_t i = 0; i < halfCount; ++i) {
      spawnAgent(width * 0.08f, width * 0.26f, height * 0.36f, height * 0.64f);
    }
    for (std::size_t i = halfCount; i < count; ++i) {
      Agent agent{Vec2{Random::range(width * 0.74f, width * 0.92f),
                       Random::range(height * 0.36f, height * 0.64f)},
                  Vec2{}, m_config.agentRadius};
      agent.velocity = Vec2{-m_config.maxSpeed * 0.35f, 0.0f};
      m_entities.push_back(agent);
    }
    break;
  }
  case CanonicalScenario::BottleneckDoorway: {
    m_goal = Vec2{width * 0.9f, height * 0.5f};
    const float doorwayY = height * 0.5f;
    const float doorwayHalfGap = 42.0f;
    for (float y = 40.0f; y < height - 40.0f; y += 38.0f) {
      if (y > doorwayY - doorwayHalfGap && y < doorwayY + doorwayHalfGap) {
        continue;
      }
      m_obstacleMask.paintCircle(Vec2{width * 0.5f, y}, 16.0f,
                                 simfw::simulation::ObstaclePaintMode::Block);
    }
    for (std::size_t i = 0; i < count; ++i) {
      spawnAgent(width * 0.08f, width * 0.30f, height * 0.12f, height * 0.88f);
    }
    break;
  }
  case CanonicalScenario::EvacuationBlockedExits: {
    m_goal = Vec2{width * 0.5f, height * 0.06f};
    m_obstacleMask.paintCircle(Vec2{width * 0.20f, height * 0.08f}, 42.0f,
                               simfw::simulation::ObstaclePaintMode::Block);
    m_obstacleMask.paintCircle(Vec2{width * 0.80f, height * 0.08f}, 42.0f,
                               simfw::simulation::ObstaclePaintMode::Block);
    m_obstacleMask.paintCircle(Vec2{width * 0.50f, height * 0.92f}, 68.0f,
                               simfw::simulation::ObstaclePaintMode::Block);
    for (std::size_t i = 0; i < count; ++i) {
      spawnAgent(width * 0.14f, width * 0.86f, height * 0.38f, height * 0.92f);
    }
    break;
  }
  case CanonicalScenario::MovingHazardRegion: {
    m_goal = Vec2{width * 0.92f, height * 0.5f};
    m_obstacleMask.paintCircle(Vec2{width * 0.45f, height * 0.5f}, 55.0f,
                               simfw::simulation::ObstaclePaintMode::Block);
    m_obstacleMask.paintCircle(Vec2{width * 0.62f, height * 0.5f}, 48.0f,
                               simfw::simulation::ObstaclePaintMode::Block);
    for (std::size_t i = 0; i < count; ++i) {
      spawnAgent(width * 0.08f, width * 0.32f, height * 0.14f, height * 0.86f);
    }
    break;
  }
  }

  m_previousAgents.clear();
  m_previousAgents.reserve(m_entities.size());
  m_config.entityCount = m_entities.size();
  m_activeScenario = scenario;
  m_scenarioTime = 0.0f;
  buildFlowField();
}

void Simulation::buildSpatialIndexes() {
  m_agentGrid.setCellSize(m_config.gridCellSize);
  if (m_config.execution.useSpatialGrid) {
    m_agentGrid.build(m_previousAgents, agentPosition);
    m_stats.occupiedGridCells = m_agentGrid.getCells().size();
  } else
    m_agentGrid.clear();
}

bool Simulation::isBlockedWorld(Vec2 worldPos) const {
  return simfw::simulation::isBlockedWorld(m_obstacleMask, worldPos);
}

Vec2 Simulation::resolveObstacleCollision(Vec2 previousPos, Vec2 proposedPos,
                                          float radius) const {
  return simfw::simulation::resolveObstacleCollision(
      m_obstacleMask, previousPos, proposedPos, radius);
}

void Simulation::buildFlowField() {
  m_gridWidth =
      static_cast<std::size_t>(m_config.width / m_config.gridCellSize) + 1;
  m_gridHeight =
      static_cast<std::size_t>(m_config.height / m_config.gridCellSize) + 1;
  const std::size_t count = m_gridWidth * m_gridHeight;
  m_costField.assign(count, 1.0f);
  m_integrationField.assign(count, std::numeric_limits<float>::infinity());
  for (std::size_t y = 0; y < m_gridHeight; ++y) {
    for (std::size_t x = 0; x < m_gridWidth; ++x) {
      const float minWorldX = static_cast<float>(x) * m_config.gridCellSize;
      const float maxWorldX =
          std::min(m_config.width, minWorldX + m_config.gridCellSize);
      const float minWorldY = static_cast<float>(y) * m_config.gridCellSize;
      const float maxWorldY =
          std::min(m_config.height, minWorldY + m_config.gridCellSize);

      const int minX = std::max(0, static_cast<int>(std::floor(minWorldX)));
      const int maxX = std::min(m_obstacleMask.width() - 1,
                                static_cast<int>(std::ceil(maxWorldX)) - 1);
      const int minY = std::max(0, static_cast<int>(std::floor(minWorldY)));
      const int maxY = std::min(m_obstacleMask.height() - 1,
                                static_cast<int>(std::ceil(maxWorldY)) - 1);

      bool blocked = false;
      for (int py = minY; py <= maxY && !blocked; ++py) {
        for (int px = minX; px <= maxX; ++px) {
          if (m_obstacleMask.isBlocked(px, py)) {
            blocked = true;
            break;
          }
        }
      }

      if (blocked) {
        m_costField[y * m_gridWidth + x] = 255.0f;
      }
    }
  }
  auto goalX = std::clamp(static_cast<int>(m_goal.x / m_config.gridCellSize), 0,
                          static_cast<int>(m_gridWidth - 1));
  auto goalY = std::clamp(static_cast<int>(m_goal.y / m_config.gridCellSize), 0,
                          static_cast<int>(m_gridHeight - 1));
  std::queue<std::size_t> q;
  std::size_t gi = static_cast<std::size_t>(goalY) * m_gridWidth +
                   static_cast<std::size_t>(goalX);
  m_integrationField[gi] = 0.0f;
  q.push(gi);
  constexpr int dx[4] = {1, -1, 0, 0};
  constexpr int dy[4] = {0, 0, 1, -1};
  while (!q.empty()) {
    std::size_t i = q.front();
    q.pop();
    std::size_t x = i % m_gridWidth;
    std::size_t y = i / m_gridWidth;
    for (int k = 0; k < 4; ++k) {
      int nx = static_cast<int>(x) + dx[k], ny = static_cast<int>(y) + dy[k];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(m_gridWidth) ||
          ny >= static_cast<int>(m_gridHeight))
        continue;
      std::size_t ni = static_cast<std::size_t>(ny) * m_gridWidth +
                       static_cast<std::size_t>(nx);
      if (m_costField[ni] >= 255.0f)
        continue;
      float nd = m_integrationField[i] + m_costField[ni];
      if (nd < m_integrationField[ni]) {
        m_integrationField[ni] = nd;
        q.push(ni);
      }
    }
  }
}

Vec2 Simulation::sampleFlow(Vec2 worldPos) const {
  if (m_integrationField.empty() || m_gridWidth == 0 || m_gridHeight == 0) {
    return Vec2{};
  }

  const int cx =
      std::clamp(static_cast<int>(worldPos.x / m_config.gridCellSize), 0,
                 static_cast<int>(m_gridWidth - 1));
  const int cy =
      std::clamp(static_cast<int>(worldPos.y / m_config.gridCellSize), 0,
                 static_cast<int>(m_gridHeight - 1));

  const std::size_t centerIndex =
      static_cast<std::size_t>(cy) * m_gridWidth + static_cast<std::size_t>(cx);
  const float centerIntegration = m_integrationField[centerIndex];
  if (!std::isfinite(centerIntegration)) {
    return Vec2{};
  }

  float bestIntegration = centerIntegration;
  Vec2 bestDirection{};
  constexpr int offsets[8][2] = {{1, 0}, {-1, 0}, {0, 1},  {0, -1},
                                 {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  for (const auto &offset : offsets) {
    const int nx = cx + offset[0];
    const int ny = cy + offset[1];
    if (nx < 0 || ny < 0 || nx >= static_cast<int>(m_gridWidth) ||
        ny >= static_cast<int>(m_gridHeight)) {
      continue;
    }

    const std::size_t neighborIndex =
        static_cast<std::size_t>(ny) * m_gridWidth +
        static_cast<std::size_t>(nx);
    const float integration = m_integrationField[neighborIndex];
    if (!std::isfinite(integration) || integration >= bestIntegration) {
      continue;
    }

    bestIntegration = integration;
    bestDirection =
        Vec2{static_cast<float>(offset[0]), static_cast<float>(offset[1])};
  }

  return bestDirection.normalized();
}

void Simulation::updateAgents(float dt) {
  auto worker = [&](std::size_t begin, std::size_t end, Scratch &s,
                    SimulationStats &st) {
    for (std::size_t i = begin; i < end; ++i) {
      simfw::simulation::collectCandidates(
          m_agentGrid, m_previousAgents[i].position, m_config.separationRadius,
          simfw::simulation::makeSpatialQueryOptionsExcluding(
              m_config.execution.useSpatialGrid, m_previousAgents.size(), i),
          s.candidates);
      st.neighborCandidates += s.candidates.size();
      BehaviorContext ctx{m_config,    m_previousAgents, m_integrationField,
                          m_gridWidth, m_gridHeight,     {s.candidates},
                          st};
      Vec2 acc = computeAcceleration(i, ctx, m_behaviors);
      Agent &a = m_entities[i];
      const Agent &p = m_previousAgents[i];
      a.velocity = limitLength(p.velocity + acc * dt, m_config.maxSpeed);
      const Vec2 proposed = p.position + a.velocity * dt;
      Vec2 collisionStart = p.position;
      if (simfw::simulation::isBlockedWorld(m_obstacleMask, collisionStart)) {
        const Vec2 backstepDir = a.velocity.lengthSquared() > 1e-6f
            ? (a.velocity.normalized() * -1.0f)
            : Vec2{-1.0f, 0.0f};
        collisionStart = p.position + backstepDir * (a.radius + 1.0f);
      }
      const simfw::simulation::ObstacleCollisionResult collision =
          simfw::simulation::resolveObstacleCollisionWithNormal(
              m_obstacleMask, collisionStart, proposed, a.radius);
      a.position = collision.position;
      if (collision.collided && collision.normal.lengthSquared() > 1e-6f) {
        const float normalSpeed = a.velocity.dot(collision.normal);
        if (normalSpeed < 0.0f) {
          constexpr float restitution = 0.35f;
          a.velocity -= collision.normal * ((1.0f + restitution) * normalSpeed);
        }
      }
      a.position.x =
          std::clamp(a.position.x, a.radius, m_config.width - a.radius);
      a.position.y =
          std::clamp(a.position.y, a.radius, m_config.height - a.radius);
      if ((a.position - m_goal).length() <= m_config.goalRadius)
        ++st.reachedGoalCount;
    }
  };
  simfw::runParallelUpdate<ThreadPool, Scratch, SimulationStats>(
      m_threadPool.get(), m_entities.size(), MinItemsPerParallelTask,
      m_config.execution.useParallelUpdate, worker,
      [this](const SimulationStats &s) {
        simfw::sumStatsMembers(m_stats, s, &SimulationStats::neighborCandidates,
                               &SimulationStats::neighborChecks,
                               &SimulationStats::reachedGoalCount);
      });
}

void Simulation::update(float dt) {
  beginFrame();
  m_previousAgents = m_entities;
  buildSpatialIndexes();
  buildFlowField();
  updateAgents(dt);
}

} // namespace crowd_cpu
