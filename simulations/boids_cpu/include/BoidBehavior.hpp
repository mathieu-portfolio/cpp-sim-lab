#pragma once

#include "Boid.hpp"
#include <vector>

Vec2 computeAlignment(const Boid& boid, const std::vector<Boid>& boids, float radius, float maxSpeed);
Vec2 computeCohesion(const Boid& boid, const std::vector<Boid>& boids, float radius, float maxSpeed);
Vec2 computeSeparation(const Boid& boid, const std::vector<Boid>& boids, float radius, float maxSpeed);

Vec2 limitLength(Vec2 v, float maxLength);
Vec2 wrapPosition(Vec2 pos, float width, float height);