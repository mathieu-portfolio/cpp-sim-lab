#pragma once

#include "Particle.hpp"

#include <cstddef>
#include <vector>

struct SimulationStats {
    std::size_t particleCount = 0;
    std::size_t maxParticleCount = 0;
};

class Simulation {
public:
    explicit Simulation(std::size_t maxParticleCount = 5000);

    void update(float dt);
    void spawn(const Vec2& pos);
    void clear();

    const std::vector<Particle>& getParticles() const { return m_particles; }
    SimulationStats getStats() const;

private:
    std::vector<Particle> m_particles;
    std::size_t m_maxParticleCount;
};
