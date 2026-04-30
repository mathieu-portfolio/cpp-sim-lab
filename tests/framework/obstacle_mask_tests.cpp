#include <simulation/ObstacleMask.hpp>

#include <gtest/gtest.h>

TEST(ObstacleMaskTests, PaintBlockCircleBlocksCells) {
    simfw::simulation::ObstacleMask mask(32, 32);
    EXPECT_TRUE(mask.paintCircle(Vec2{16.0f, 16.0f}, 4.0f, simfw::simulation::ObstaclePaintMode::Block));
    EXPECT_TRUE(mask.isBlocked(16, 16));
}

TEST(ObstacleMaskTests, PaintEraseCircleClearsCells) {
    simfw::simulation::ObstacleMask mask(32, 32);
    mask.paintCircle(Vec2{16.0f, 16.0f}, 4.0f, simfw::simulation::ObstaclePaintMode::Block);
    EXPECT_TRUE(mask.paintCircle(Vec2{16.0f, 16.0f}, 4.0f, simfw::simulation::ObstaclePaintMode::Erase));
    EXPECT_FALSE(mask.isBlocked(16, 16));
}

TEST(ObstacleMaskTests, OutOfBoundsPaintDoesNotCrash) {
    simfw::simulation::ObstacleMask mask(16, 16);
    EXPECT_TRUE(mask.paintCircle(Vec2{-4.0f, -4.0f}, 8.0f, simfw::simulation::ObstaclePaintMode::Block));
}
