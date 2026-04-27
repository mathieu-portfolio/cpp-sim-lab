#include <math/Vec2.hpp>

#include <gtest/gtest.h>

TEST(Vec2, AddsComponents) {
    Vec2 a{1.0f, 2.0f};
    Vec2 b{3.0f, 4.0f};

    Vec2 result = a + b;

    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST(Vec2, SubtractsComponents) {
    Vec2 a{5.0f, 7.0f};
    Vec2 b{2.0f, 3.0f};

    Vec2 result = a - b;

    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
}

TEST(Vec2, MultipliesByScalar) {
    Vec2 v{2.0f, 3.0f};

    Vec2 result = v * 4.0f;

    EXPECT_FLOAT_EQ(result.x, 8.0f);
    EXPECT_FLOAT_EQ(result.y, 12.0f);
}

TEST(Vec2, ComputesLength) {
    Vec2 v{3.0f, 4.0f};

    EXPECT_FLOAT_EQ(v.length(), 5.0f);
}

TEST(Vec2, NormalizesToUnitLength) {
    Vec2 v{3.0f, 4.0f};

    Vec2 n = v.normalized();

    EXPECT_NEAR(n.length(), 1.0f, 0.0001f);
}

TEST(Vec2, ComputesDotProduct) {
    Vec2 a{1.0f, 2.0f};
    Vec2 b{3.0f, 4.0f};

    EXPECT_FLOAT_EQ(Vec2::dot(a, b), 11.0f);
}