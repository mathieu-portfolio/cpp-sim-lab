#include "Particle.hpp"
#include "SpatialGrid.hpp"

#include <BenchTimer.hpp>
#include <math/Vec2.hpp>
#include <random/Random.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

namespace {
constexpr float WorldWidth = 800.0f;
constexpr float WorldHeight = 800.0f;
constexpr float ParticleRadius = 4.0f;
constexpr float Epsilon = 0.0001f;

struct BenchResult {
    double totalMsPerStep = 0.0;
    double buildMsPerStep = 0.0;
    double queryCollisionMsPerStep = 0.0;
    std::size_t checksPerStep = 0;
    std::size_t resolvedPerStep = 0;
};

class FixedGrid {
public:
    FixedGrid(float width, float height, float cellSize)
        : m_width(width),
          m_height(height),
          m_cellSize(cellSize),
          m_cellsX(static_cast<int>(std::ceil(width / cellSize))),
          m_cellsY(static_cast<int>(std::ceil(height / cellSize))),
          m_cells(static_cast<std::size_t>(m_cellsX * m_cellsY)) {}

    void clear() {
        for (auto& cell : m_cells) {
            cell.clear();
        }
    }

    void build(const std::vector<Particle>& particles) {
        clear();

        for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
            insert(i, particles[i].position);
        }
    }

    void queryNeighbors(const Vec2& position, std::vector<int>& outIndices) const {
        const CellCoord base = toCell(position);

        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int x = base.x + dx;
                const int y = base.y + dy;

                if (!isValidCell(x, y)) {
                    continue;
                }

                const auto& cell = m_cells[index(x, y)];
                outIndices.insert(outIndices.end(), cell.begin(), cell.end());
            }
        }
    }

private:
    struct CellCoord {
        int x = 0;
        int y = 0;
    };

    float m_width = 0.0f;
    float m_height = 0.0f;
    float m_cellSize = 1.0f;
    int m_cellsX = 0;
    int m_cellsY = 0;
    std::vector<std::vector<int>> m_cells;

    void insert(int particleIndex, const Vec2& position) {
        const CellCoord cell = toCell(position);
        m_cells[index(cell.x, cell.y)].push_back(particleIndex);
    }

    CellCoord toCell(const Vec2& position) const {
        const int x = std::clamp(
            static_cast<int>(position.x / m_cellSize),
            0,
            m_cellsX - 1
        );

        const int y = std::clamp(
            static_cast<int>(position.y / m_cellSize),
            0,
            m_cellsY - 1
        );

        return CellCoord{x, y};
    }

    bool isValidCell(int x, int y) const {
        return x >= 0 && x < m_cellsX &&
               y >= 0 && y < m_cellsY;
    }

    std::size_t index(int x, int y) const {
        return static_cast<std::size_t>(y * m_cellsX + x);
    }
};

static std::vector<Particle> makeParticles(std::size_t count) {
    std::vector<Particle> particles;
    particles.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        particles.push_back({
            Vec2{
                Random::range(0.0f, WorldWidth),
                Random::range(0.0f, WorldHeight)
            },
            Vec2{},
            ParticleRadius
        });
    }

    return particles;
}

static bool resolvePair(Particle& a, Particle& b) {
    Vec2 delta = b.position - a.position;
    float dist = delta.length();
    const float minDist = a.radius + b.radius;

    if (dist <= Epsilon || dist >= minDist) {
        return false;
    }

    const Vec2 normal = delta * (1.0f / dist);
    const float penetration = minDist - dist;
    const Vec2 correction = normal * (penetration * 0.5f);

    a.position -= correction;
    b.position += correction;

    return true;
}

template <typename Grid>
static BenchResult benchGrid(std::size_t count, float cellSize, int steps) {
    auto particles = makeParticles(count);

    Grid grid{cellSize};
    std::vector<int> candidates;

    std::size_t totalChecks = 0;
    std::size_t totalResolved = 0;

    double buildMs = 0.0;
    double queryCollisionMs = 0.0;

    const double totalMs = bench::measureMs([&]() {
        for (int step = 0; step < steps; ++step) {
            buildMs += bench::measureMs([&]() {
                grid.build(particles);
            });

            queryCollisionMs += bench::measureMs([&]() {
                for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
                    candidates.clear();
                    grid.queryNeighbors(particles[i].position, candidates);

                    for (int j : candidates) {
                        if (j <= i) {
                            continue;
                        }

                        ++totalChecks;

                        if (resolvePair(particles[i], particles[j])) {
                            ++totalResolved;
                        }
                    }
                }
            });
        }
    });

    return {
        totalMs / steps,
        buildMs / steps,
        queryCollisionMs / steps,
        totalChecks / static_cast<std::size_t>(steps),
        totalResolved / static_cast<std::size_t>(steps)
    };
}

static BenchResult benchSpatialGrid(std::size_t count, float cellSize, int steps) {
    return benchGrid<SpatialGrid>(count, cellSize, steps);
}

static BenchResult benchFixedGrid(std::size_t count, float cellSize, int steps) {
    auto particles = makeParticles(count);

    FixedGrid grid{WorldWidth, WorldHeight, cellSize};
    std::vector<int> candidates;

    std::size_t totalChecks = 0;
    std::size_t totalResolved = 0;

    double buildMs = 0.0;
    double queryCollisionMs = 0.0;

    const double totalMs = bench::measureMs([&]() {
        for (int step = 0; step < steps; ++step) {
            buildMs += bench::measureMs([&]() {
                grid.build(particles);
            });

            queryCollisionMs += bench::measureMs([&]() {
                for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
                    candidates.clear();
                    grid.queryNeighbors(particles[i].position, candidates);

                    for (int j : candidates) {
                        if (j <= i) {
                            continue;
                        }

                        ++totalChecks;

                        if (resolvePair(particles[i], particles[j])) {
                            ++totalResolved;
                        }
                    }
                }
            });
        }
    });

    return {
        totalMs / steps,
        buildMs / steps,
        queryCollisionMs / steps,
        totalChecks / static_cast<std::size_t>(steps),
        totalResolved / static_cast<std::size_t>(steps)
    };
}
}

int main() {
    constexpr int steps = 60;
    constexpr float cellSize = 16.0f;

    const std::vector<std::size_t> particleCounts{
        100,
        500,
        1000,
        2000,
        5000
    };

    std::cout
        << "particles,cell_size,"
        << "unordered_total_ms,unordered_build_ms,unordered_query_collision_ms,"
        << "fixed_total_ms,fixed_build_ms,fixed_query_collision_ms,"
        << "unordered_checks,fixed_checks,"
        << "unordered_resolved,fixed_resolved,"
        << "speedup\n";

    for (std::size_t count : particleCounts) {
        const BenchResult unorderedResult =
            benchSpatialGrid(count, cellSize, steps);

        const BenchResult fixedResult =
            benchFixedGrid(count, cellSize, steps);

        const double speedup =
            unorderedResult.totalMsPerStep / fixedResult.totalMsPerStep;

        std::cout << count << ","
                  << cellSize << ","
                  << unorderedResult.totalMsPerStep << ","
                  << unorderedResult.buildMsPerStep << ","
                  << unorderedResult.queryCollisionMsPerStep << ","
                  << fixedResult.totalMsPerStep << ","
                  << fixedResult.buildMsPerStep << ","
                  << fixedResult.queryCollisionMsPerStep << ","
                  << unorderedResult.checksPerStep << ","
                  << fixedResult.checksPerStep << ","
                  << unorderedResult.resolvedPerStep << ","
                  << fixedResult.resolvedPerStep << ","
                  << speedup << "\n";
    }

    return 0;
}
