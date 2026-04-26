#include "Simulation.hpp"

#include <random/Random.hpp>

Simulation::Simulation(std::size_t maxParticleCount)
    : m_maxParticleCount(maxParticleCount) {
    m_particles.reserve(m_maxParticleCount);
}

void Simulation::update(float dt) {
    const Vec2 gravity{0.0f, 200.0f};

    for (auto& p : m_particles) {
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

void Simulation::spawn(const Vec2& pos) {
    constexpr int spawnCount = 10;

    for (int i = 0; i < spawnCount && m_particles.size() < m_maxParticleCount; ++i) {
        m_particles.push_back({
            pos,
            Vec2{
                Random::range(-100.0f, 100.0f),
                Random::range(-150.0f, -50.0f)
            }
        });
    }
}

void Simulation::clear() {
    m_particles.clear();
}

SimulationStats Simulation::getStats() const {
    return SimulationStats{
        m_particles.size(),
        m_maxParticleCount
    };
}
