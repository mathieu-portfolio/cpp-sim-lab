#include "BoidBehavior.hpp"

Vec2 computeAlignment(const Boid&, const std::vector<Boid>&, float) { return {}; }
Vec2 computeCohesion(const Boid&, const std::vector<Boid>&, float) { return {}; }
Vec2 computeSeparation(const Boid&, const std::vector<Boid>&, float) { return {}; }

Vec2 limitLength(Vec2 v, float maxLength) {
    float len = v.length();
    if (len > maxLength && len > 0.0f) {
        return v * (maxLength / len);
    }
    return v;
}

Vec2 wrapPosition(Vec2 pos, float w, float h) {
    if (pos.x < 0) pos.x += w;
    if (pos.x > w) pos.x -= w;
    if (pos.y < 0) pos.y += h;
    if (pos.y > h) pos.y -= h;
    return pos;
}
