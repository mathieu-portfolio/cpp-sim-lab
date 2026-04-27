#pragma once

#include "Boid.hpp"
#include <vector>

struct SimulationConfig {
    float width = 800.0f;
    float height = 800.0f;

    float perceptionRadius = 50.0f;
    float separationRadius = 20.0f;

    float maxSpeed = 120.0f;
    float maxForce = 80.0f;

    float alignmentWeight = 1.0f;
    float cohesionWeight = 0.5f;
    float separationWeight = 1.5f;

    std::size_t boidCount = 200;
};

class Simulation {
public:
    explicit Simulation(SimulationConfig config = {});
    void update(float dt);
    void reset();

    const std::vector<Boid>& getBoids() const { return m_boids; }
    const SimulationConfig& getConfig() const { return m_config; }

private:
    std::vector<Boid> m_boids;
    SimulationConfig m_config;
};
