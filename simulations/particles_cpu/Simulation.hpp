#pragma once

#include "Particle.hpp"
#include <vector>

struct SimulationStats {
    std::size_t particleCount = 0;
};

class Simulation {
public:
    void update(float dt);
    void spawn(const Vec2& pos);
    void clear();

    const std::vector<Particle>& getParticles() const { return m_particles; }
    SimulationStats getStats() const;

private:
    std::vector<Particle> m_particles;
};
