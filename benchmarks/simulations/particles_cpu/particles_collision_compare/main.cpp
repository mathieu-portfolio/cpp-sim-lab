#include "Particle.hpp"
#include "SpatialGrid.hpp"

#include <BenchTimer.hpp>
#include <math/Vec2.hpp>
#include <random/Random.hpp>

#include <iostream>
#include <vector>

struct BenchResult {
    double msPerStep = 0.0;
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
    float minDist = a.radius + b.radius;

    if (dist <= 0.0001f || dist >= minDist) {
        return false;
    }

    Vec2 normal = delta * (1.0f / dist);
    float penetration = minDist - dist;
    Vec2 correction = normal * (penetration * 0.5f);

    a.position -= correction;
    b.position += correction;

    return true;
}

static BenchResult benchNaive(std::size_t count, int steps) {
    auto particles = makeParticles(count);

    std::size_t totalChecks = 0;
    std::size_t totalResolved = 0;

    const double totalMs = bench::measureMs([&]() {
        for (int step = 0; step < steps; ++step) {
            for (std::size_t i = 0; i < particles.size(); ++i) {
                for (std::size_t j = i + 1; j < particles.size(); ++j) {
                    ++totalChecks;

                    if (resolvePair(particles[i], particles[j])) {
                        ++totalResolved;
                    }
                }
            }
        }
    });

    return {
        totalMs / steps,
        totalChecks / static_cast<std::size_t>(steps),
        totalResolved / static_cast<std::size_t>(steps)
    };
}

static BenchResult benchGrid(std::size_t count, int steps) {
    auto particles = makeParticles(count);

    SpatialGrid grid{16.0f};
    std::vector<int> candidates;

    std::size_t totalChecks = 0;
    std::size_t totalResolved = 0;

    const double totalMs = bench::measureMs([&]() {
        for (int step = 0; step < steps; ++step) {
            grid.build(particles);

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
        }
    });

    return {
        totalMs / steps,
        totalChecks / static_cast<std::size_t>(steps),
        totalResolved / static_cast<std::size_t>(steps)
    };
}

int main() {
    constexpr int steps = 60;

    std::cout << "particles,naive_ms,grid_ms,speedup,naive_checks,grid_checks,naive_resolved,grid_resolved\n";

    for (std::size_t count : {100ull, 500ull, 1000ull, 2000ull, 5000ull}) {
        BenchResult naive = benchNaive(count, steps);
        BenchResult grid = benchGrid(count, steps);

        double speedup = naive.msPerStep / grid.msPerStep;

        std::cout << count << ","
                  << naive.msPerStep << ","
                  << grid.msPerStep << ","
                  << speedup << ","
                  << naive.checksPerStep << ","
                  << grid.checksPerStep << ","
                  << naive.resolvedPerStep << ","
                  << grid.resolvedPerStep << "\n";
    }

    return 0;
}
