#pragma once

#include <math/Vec2.hpp>

namespace predator_prey_cpu {

enum class AgentType {
    Prey,
    Predator
};

struct Boid {
    Vec2 position;
    Vec2 velocity;
    AgentType type = AgentType::Prey;
    float energy = 1.0f;
    bool alive = true;
};

} // namespace predator_prey_cpu
