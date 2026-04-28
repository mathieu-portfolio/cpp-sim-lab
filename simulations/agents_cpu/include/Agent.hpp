#pragma once

#include <math/Vec2.hpp>

namespace agents_cpu {

struct Agent {
    Vec2 position;
    Vec2 velocity;
    Vec2 target;

    float radius = 4.0f;
};

} // namespace agents_cpu
