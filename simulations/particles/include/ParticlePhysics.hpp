#pragma once

#include "Particle.hpp"
#include "Simulation.hpp"

namespace particles {

bool resolveParticleCollision(
    Particle& a,
    Particle& b,
    const SimulationConfig& config
);

void solveParticleBounds(
    Particle& particle,
    const SimulationConfig& config
);

} // namespace particles
