#include <math/Vec2.hpp>

#include <cmath>
#include <iostream>

static bool nearlyEqual(float a, float b, float epsilon = 0.0001f) {
    return std::fabs(a - b) <= epsilon;
}

static void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

int main() {
    {
        Vec2 a{1.0f, 2.0f};
        Vec2 b{3.0f, 4.0f};

        Vec2 result = a + b;

        expect(nearlyEqual(result.x, 4.0f), "Vec2 addition x");
        expect(nearlyEqual(result.y, 6.0f), "Vec2 addition y");
    }

    {
        Vec2 a{5.0f, 7.0f};
        Vec2 b{2.0f, 3.0f};

        Vec2 result = a - b;

        expect(nearlyEqual(result.x, 3.0f), "Vec2 subtraction x");
        expect(nearlyEqual(result.y, 4.0f), "Vec2 subtraction y");
    }

    {
        Vec2 v{2.0f, 3.0f};

        Vec2 result = v * 4.0f;

        expect(nearlyEqual(result.x, 8.0f), "Vec2 scalar multiplication x");
        expect(nearlyEqual(result.y, 12.0f), "Vec2 scalar multiplication y");
    }

    {
        Vec2 v{3.0f, 4.0f};

        expect(nearlyEqual(v.length(), 5.0f), "Vec2 length");
    }

    {
        Vec2 v{3.0f, 4.0f};
        Vec2 n = v.normalized();

        expect(nearlyEqual(n.length(), 1.0f), "Vec2 normalized length");
    }

    {
        Vec2 a{1.0f, 2.0f};
        Vec2 b{3.0f, 4.0f};

        expect(nearlyEqual(Vec2::dot(a, b), 11.0f), "Vec2 dot product");
    }

    std::cout << "Vec2 tests passed\n";
    return 0;
}
