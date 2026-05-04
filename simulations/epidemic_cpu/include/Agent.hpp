#pragma once

#include <math/Vec2.hpp>

namespace epidemic_cpu {

enum class InfectionState {
    Susceptible,
    Infected,
    Recovered
};

struct Agent {
    Vec2 position{};
    Vec2 velocity{};
    float radius = 3.0f;
    InfectionState infectionState = InfectionState::Susceptible;
    float infectedSeconds = 0.0f;
    float exposureSeconds = 0.0f;
};

} // namespace epidemic_cpu
