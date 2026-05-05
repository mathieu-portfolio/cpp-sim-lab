#pragma once

#include <math/Vec2.hpp>

namespace crowd {

struct Obstacle {
    Vec2 position;
    float radius = 24.0f;
};

} // namespace crowd
