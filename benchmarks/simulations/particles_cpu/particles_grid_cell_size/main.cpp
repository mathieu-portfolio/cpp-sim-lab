#include "Particle.hpp"
#include "SpatialGrid.hpp"

#include <BenchTimer.hpp>
#include <math/Vec2.hpp>
#include <random/Random.hpp>

#include <cstddef>
#include <iostream>
#include <vector>

struct BenchResult {
    double totalMsPerStep = 0.0;
    double buildMsPerStep = 0.0;
    double queryCollisionMsPerStep = 0.0;
    std::size_t checksPerStep = 0;
    std::size_t resolvedPerStep = 0;
};

static std::vector<Particle> makeParticles(std::size_t count) {
    std::vector<Particle> particles;
    particles.reserve(count);

    for (std::size_t i = 0; i < count; ++i) {
        particles.push_back({
            Vec2{
                Random::range(0.0f, 800.0f),
                Random::range(0.0f, 800.0f)
            },
            Vec2{},
            4.0f
        });
    }

    return particles;
}

static bool resolvePair(Particle& a, Particle& b) {
    Vec2 delta = b.position - a.position;
    float dist = delta.length();
    const float minDist = a.radius + b.radius;

    if (dist <= 0.0001f || dist >= minDist) {
        return false;
    }

    const Vec2 normal = delta * (1.0f / dist);
    const float penetration = minDist - dist;
    const Vec2 correction = normal * (penetration * 0.5f);

    a.position -= correction;
    b.position += correction;

    return true;
}

static BenchResult benchGrid(std::size_t count, float cellSize, int steps) {
    auto particles = makeParticles(count);

    SpatialGrid grid{cellSize};
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

int main() {
    constexpr int steps = 60;

    const std::vector<std::size_t> particleCounts{
        100,
        500,
        1000,
        2000,
        5000
    };

    const std::vector<float> cellSizes{
        8.0f,
        12.0f,
        16.0f,
        24.0f,
        32.0f,
        48.0f,
        64.0f
    };

    std::cout
        << "particles,cell_size,total_ms,build_ms,query_collision_ms,checks,resolved\n";

    for (std::size_t count : particleCounts) {
        for (float cellSize : cellSizes) {
            BenchResult result = benchGrid(count, cellSize, steps);

            std::cout << count << ","
                      << cellSize << ","
                      << result.totalMsPerStep << ","
                      << result.buildMsPerStep << ","
                      << result.queryCollisionMsPerStep << ","
                      << result.checksPerStep << ","
                      << result.resolvedPerStep << "\n";
        }
    }

    return 0;
}
