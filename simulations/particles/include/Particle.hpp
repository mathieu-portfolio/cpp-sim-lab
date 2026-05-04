#pragma once

#include <math/Vec2.hpp>

namespace particles {

struct Particle {
    Vec2 position;
    Vec2 velocity;
    float radius = 4.0f;
};

} // namespace particles
