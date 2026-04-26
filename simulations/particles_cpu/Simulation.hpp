#pragma once

#include "Particle.hpp"
#include <vector>

class Simulation {
public:
    void update(float dt);
    void spawn(const Vec2& pos);
    void clear();

    const std::vector<Particle>& getParticles() const { return m_particles; }

private:
    std::vector<Particle> m_particles;
};
