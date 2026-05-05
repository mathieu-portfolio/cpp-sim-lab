#pragma once

#include <cstdint>

namespace sand_cpu {

enum class Material : std::uint8_t {
    Empty = 0,
    Sand,
    Water,
    Smoke
};

struct Cell {
    Material material = Material::Empty;
    std::uint8_t life = 0;
    float vx = 0.0f;
    float vy = 0.0f;
};

} // namespace sand_cpu
