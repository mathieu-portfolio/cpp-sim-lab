#include "Particle.hpp"

#include <BenchTimer.hpp>
#include <ProgressBar.hpp>
#include <BenchmarkRandom.hpp>
#include <math/Vec2.hpp>
#include <random/Random.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

using namespace particles_cpu;

namespace {
constexpr std::uint32_t BaseSeed = 1337u;

Vec2 particlePosition(const Particle& particle) {
    return particle.position;
}
}

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

static BenchResult benchNaive(std::size_t count, int steps, std::uint32_t seed) {
    Random::seed(seed);
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

static BenchResult benchGrid(std::size_t count, int steps, std::uint32_t seed) {
    Random::seed(seed);
    auto particles = makeParticles(count);

    simfw::SpatialHashGrid<int> grid{16.0f};
    std::vector<int> candidates;

    std::size_t totalChecks = 0;
    std::size_t totalResolved = 0;

    const double totalMs = bench::measureMs([&]() {
        for (int step = 0; step < steps; ++step) {
            grid.build(particles, particlePosition);

            for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
                candidates.clear();
                grid.queryCellsAround(particles[i].position, 1, candidates);

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

    constexpr std::size_t totalCases = 5;
    bench::ProgressBar progress(totalCases);

    for (std::size_t count : {100ull, 500ull, 1000ull, 2000ull, 5000ull}) {
        const std::uint32_t seed = bench::seedFor(BaseSeed, count);

        BenchResult naive = benchNaive(count, steps, seed);
        BenchResult grid = benchGrid(count, steps, seed);

        double speedup = naive.msPerStep / grid.msPerStep;

        progress.advance();
        std::cout << count << ","
                  << naive.msPerStep << ","
                  << grid.msPerStep << ","
                  << speedup << ","
                  << naive.checksPerStep << ","
                  << grid.checksPerStep << ","
                  << naive.resolvedPerStep << ","
                  << grid.resolvedPerStep << "\n";
    }

    progress.finish();

    return 0;
}
