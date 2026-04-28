#pragma once

#include <math/Vec2.hpp>

namespace agents_cpu {

struct Obstacle {
    Vec2 position;
    float radius = 24.0f;
};

} // namespace agents_cpu
