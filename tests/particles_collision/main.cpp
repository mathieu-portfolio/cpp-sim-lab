#include "ParticlePhysics.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

static bool nearlyEqual(float a, float b, float epsilon = 0.0001f) {
    return std::fabs(a - b) <= epsilon;
}

static void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

int main() {
    SimulationConfig config;
    config.width = 100.0f;
    config.height = 100.0f;
    config.bounce = -0.5f;
    config.restitution = 1.0f;

    {
        Particle a{{10.0f, 10.0f}, {}, 4.0f};
        Particle b{{30.0f, 10.0f}, {}, 4.0f};

        const bool resolved = resolveParticleCollision(a, b, config);

        expect(!resolved, "non-overlapping particles should not resolve");
        expect(nearlyEqual(a.position.x, 10.0f), "non-overlap keeps particle A x");
        expect(nearlyEqual(b.position.x, 30.0f), "non-overlap keeps particle B x");
    }

    {
        Particle a{{10.0f, 10.0f}, {}, 4.0f};
        Particle b{{16.0f, 10.0f}, {}, 4.0f};

        const bool resolved = resolveParticleCollision(a, b, config);

        expect(resolved, "overlapping particles should resolve");

        const float distance = (b.position - a.position).length();
        expect(nearlyEqual(distance, 8.0f), "resolved particles should be separated by radius sum");
    }

    {
        Particle a{{10.0f, 10.0f}, {10.0f, 0.0f}, 4.0f};
        Particle b{{16.0f, 10.0f}, {-10.0f, 0.0f}, 4.0f};

        const bool resolved = resolveParticleCollision(a, b, config);

        expect(resolved, "head-on overlapping particles should resolve");
        expect(a.velocity.x < 0.0f, "particle A should bounce backward");
        expect(b.velocity.x > 0.0f, "particle B should bounce backward");
    }

    {
        Particle p{{-5.0f, 50.0f}, {-10.0f, 0.0f}, 4.0f};

        solveParticleBounds(p, config);

        expect(nearlyEqual(p.position.x, 4.0f), "left boundary clamps particle position");
        expect(p.velocity.x > 0.0f, "left boundary reverses velocity");
    }

    {
        Particle p{{50.0f, 120.0f}, {0.0f, 10.0f}, 4.0f};

        solveParticleBounds(p, config);

        expect(nearlyEqual(p.position.y, 96.0f), "bottom boundary clamps particle position");
        expect(p.velocity.y < 0.0f, "bottom boundary reverses velocity");
    }

    std::cout << "Particle collision tests passed\n";
    return 0;
}