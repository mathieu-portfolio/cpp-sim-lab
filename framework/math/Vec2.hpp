#pragma once

#include <cmath>

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Vec2() = default;
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(const Vec2& other) const {
        return {x + other.x, y + other.y};
    }

    constexpr Vec2 operator-(const Vec2& other) const {
        return {x - other.x, y - other.y};
    }

    constexpr Vec2 operator*(float scalar) const {
        return {x * scalar, y * scalar};
    }

    constexpr Vec2 operator/(float scalar) const {
        return {x / scalar, y / scalar};
    }

    constexpr Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    constexpr Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    constexpr Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    constexpr Vec2& operator/=(float scalar) {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    float length() const {
        return std::sqrt(x * x + y * y);
    }

    float lengthSquared() const {
        return x * x + y * y;
    }

    Vec2 normalized() const {
        float len = length();
        return len > 0.0f ? Vec2{x / len, y / len} : Vec2{};
    }

    float dot(const Vec2& other) {
        return x * other.x + y * other.y;
    }

    static constexpr float dot(const Vec2& a, const Vec2& b) {
        return a.x * b.x + a.y * b.y;
    }
};