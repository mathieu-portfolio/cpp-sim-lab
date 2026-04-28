#pragma once

#include "Boid.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace boids_cpu {

Vec2 computeAlignment(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    std::span<const std::size_t> neighbors,
    float maxSpeed
);

Vec2 computeCohesion(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    std::span<const std::size_t> neighbors,
    float maxSpeed
);

Vec2 computeSeparation(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    std::span<const std::size_t> neighbors,
    float maxSpeed
);

Vec2 limitLength(Vec2 value, float maxLength);
Vec2 wrapPosition(Vec2 position, float width, float height);

} // namespace boids_cpu
