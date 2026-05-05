#pragma once

#include "Bubble.hpp"
#include "Simulation.hpp"

namespace bubbles {

bool resolveBubbleCollision(Bubble& a, Bubble& b, const SimulationConfig& config, float dt);
void applySurfaceTension(Bubble& a, Bubble& b, const SimulationConfig& config, float dt);
void solveBubbleBounds(Bubble& bubble, const SimulationConfig& config);
Bubble mergeBubbles(const Bubble& a, const Bubble& b);

} // namespace bubbles
