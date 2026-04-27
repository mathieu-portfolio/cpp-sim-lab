#pragma once

#include "Boid.hpp"

#include <cstddef>
#include <vector>

Vec2 computeAlignment(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& neighbors,
    float maxSpeed
);

Vec2 computeCohesion(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& neighbors,
    float maxSpeed
);

Vec2 computeSeparation(
    std::size_t boidIndex,
    const std::vector<Boid>& boids,
    const std::vector<std::size_t>& neighbors,
    float maxSpeed
);

Vec2 limitLength(Vec2 v, float maxLength);
Vec2 wrapPosition(Vec2 pos, float w, float h);