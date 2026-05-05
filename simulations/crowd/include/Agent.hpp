#pragma once

#include <math/Vec2.hpp>

namespace crowd {

struct Agent {
    Vec2 position;
    Vec2 velocity;
    float radius = 4.0f;
};

} // namespace crowd
