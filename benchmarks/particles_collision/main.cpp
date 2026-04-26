#include <math/Vec2.hpp>
#include <random/Random.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

struct Particle {
    Vec2 position;
    Vec2 velocity;
    float radius = 4.0f;
};

static std::size_t resolveCollisions(std::vector<Particle>& particles) {
    std::size_t resolved = 0;

    for (std::size_t i = 0; i < particles.size(); ++i) {
        for (std::size_t j = i + 1; j < particles.size(); ++j) {
            Particle& a = particles[i];
            Particle& b = particles[j];

            Vec2 delta = b.position - a.position;
            float dist = delta.length();
            float minDist = a.radius + b.radius;

            if (dist <= 0.0001f || dist >= minDist) {
                continue;
            }

            Vec2 normal = delta * (1.0f / dist);
            float penetration = minDist - dist;
            Vec2 correction = normal * (penetration * 0.5f);

            a.position -= correction;
            b.position += correction;

            ++resolved;
        }
    }

    return resolved;
}

static double runBenchmark(std::size_t particleCount, int steps) {
    std::vector<Particle> particles;
    particles.reserve(particleCount);

    for (std::size_t i = 0; i < particleCount; ++i) {
        particles.push_back({
            Vec2{
                Random::range(0.0f, 800.0f),
                Random::range(0.0f, 800.0f)
            },
            Vec2{},
            4.0f
        });
    }

    std::size_t totalResolved = 0;

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; ++i) {
        totalResolved += resolveCollisions(particles);
    }

    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();

    std::cout << particleCount << " particles | "
              << totalMs / steps << " ms/step | "
              << totalResolved << " resolved total\n";

    return totalMs;
}

int main() {
    constexpr int steps = 60;

    runBenchmark(100, steps);
    runBenchmark(500, steps);
    runBenchmark(1000, steps);
    runBenchmark(2000, steps);

    return 0;
}
