#include <math/Vec2.hpp>
#include <random/Random.hpp>

#include <chrono>
#include <iostream>
#include <vector>

struct Particle {
    Vec2 position;
    Vec2 velocity;
};

static void update(std::vector<Particle>& particles, float dt) {
    const Vec2 gravity{0.0f, 200.0f};

    for (auto& p : particles) {
        p.velocity += gravity * dt;
        p.position += p.velocity * dt;

        if (p.position.y > 800.0f) {
            p.position.y = 800.0f;
            p.velocity.y *= -0.8f;
        }

        if (p.position.x < 0.0f) {
            p.position.x = 0.0f;
            p.velocity.x *= -0.8f;
        }

        if (p.position.x > 800.0f) {
            p.position.x = 800.0f;
            p.velocity.x *= -0.8f;
        }

        p.velocity *= 0.999f;
    }
}

static double runBenchmark(std::size_t particleCount, int steps) {
    std::vector<Particle> particles;
    particles.reserve(particleCount);

    for (std::size_t i = 0; i < particleCount; ++i) {
        particles.push_back({
            Vec2{Random::range(0.0f, 800.0f), Random::range(0.0f, 800.0f)},
            Vec2{Random::range(-100.0f, 100.0f), Random::range(-100.0f, 100.0f)}
        });
    }

    constexpr float dt = 1.0f / 60.0f;

    const auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; ++i) {
        update(particles, dt);
    }

    const auto end = std::chrono::high_resolution_clock::now();

    return std::chrono::duration<double, std::milli>(end - start).count();
}

int main() {
    constexpr int steps = 300;

    for (std::size_t count : {1000ull, 10000ull, 100000ull, 1000000ull}) {
        const double totalMs = runBenchmark(count, steps);
        const double avgMs = totalMs / steps;

        std::cout << count << " particles | "
                  << avgMs << " ms/update | "
                  << totalMs << " ms total\n";
    }

    return 0;
}
