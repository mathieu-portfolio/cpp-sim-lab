#pragma once

#include "Particle.hpp"
#include "Simulation.hpp"

bool resolveParticleCollision(
    Particle& a,
    Particle& b,
    const SimulationConfig& config
);

void solveParticleBounds(
    Particle& particle,
    const SimulationConfig& config
);
