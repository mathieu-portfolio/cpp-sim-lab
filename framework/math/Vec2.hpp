#pragma once
#include <cmath>

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }

    float length() const { return std::sqrt(x*x + y*y); }

    Vec2 normalized() const {
        float l = length();
        return l > 0 ? Vec2{x/l, y/l} : Vec2{0,0};
    }

    static float dot(const Vec2& a, const Vec2& b) {
        return a.x*b.x + a.y*b.y;
    }
};
