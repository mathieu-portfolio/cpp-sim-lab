#pragma once

#include "AgentIntent.hpp"

#include <math/Vec2.hpp>

namespace agents_cpu {

struct Agent {
    Vec2 position;
    Vec2 velocity;
    Vec2 target;

    float radius = 4.0f;
    AgentIntent intent = AgentIntent::SeekTarget;
};

} // namespace agents_cpu
