#include "Simulation.hpp"
#include "BoidBehavior.hpp"
#include <random/Random.hpp>

Simulation::Simulation(SimulationConfig config)
    : m_config(config) {
    reset();
}

void Simulation::reset() {
    m_boids.clear();

    for (std::size_t i = 0; i < m_config.boidCount; ++i) {
        m_boids.push_back({
            Vec2{
                Random::range(0.0f, m_config.width),
                Random::range(0.0f, m_config.height)
            },
            Vec2{
                Random::range(-50.0f, 50.0f),
                Random::range(-50.0f, 50.0f)
            }
        });
    }
}

void Simulation::update(float dt) {
    for (auto& b : m_boids) {
        Vec2 align = computeAlignment(b, m_boids, m_config.perceptionRadius);
        Vec2 coh = computeCohesion(b, m_boids, m_config.perceptionRadius);
        Vec2 sep = computeSeparation(b, m_boids, m_config.separationRadius);

        Vec2 force =
            align * m_config.alignmentWeight +
            coh * m_config.cohesionWeight +
            sep * m_config.separationWeight;

        force = limitLength(force, m_config.maxForce);

        b.velocity += force * dt;
        b.velocity = limitLength(b.velocity, m_config.maxSpeed);
        b.position += b.velocity * dt;

        b.position = wrapPosition(b.position, m_config.width, m_config.height);
    }
}
