#pragma once

#include <math/Vec2.hpp>

namespace crowd_cpu {

struct Obstacle {
    Vec2 position;
    float radius = 24.0f;
};

} // namespace crowd_cpu
