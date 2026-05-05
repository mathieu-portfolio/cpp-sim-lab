#pragma once

#include <math/Vec2.hpp>

namespace bubbles_cpu_gpu {

struct Bubble {
    Vec2 position;
    Vec2 velocity;
    float radius = 8.0f;
    float restRadius = 8.0f;
    float age = 0.0f;
    float stress = 0.0f;
};

} // namespace bubbles_cpu_gpu
