#include <simulation/StatsReduction.hpp>

#include <gtest/gtest.h>

#include <cstddef>

namespace {

struct Stats {
    std::size_t checks = 0;
    std::size_t candidates = 0;
    std::size_t unchanged = 0;
};

} // namespace

TEST(StatsReductionTests, SumsSelectedMembers) {
    Stats target{
        10,
        20,
        30
    };

    const Stats source{
        2,
        3,
        4
    };

    simfw::sumStatsMembers(
        target,
        source,
        &Stats::checks,
        &Stats::candidates
    );

    EXPECT_EQ(target.checks, 12u);
    EXPECT_EQ(target.candidates, 23u);
    EXPECT_EQ(target.unchanged, 30u);
}

TEST(StatsReductionTests, SupportsEmptyMemberList) {
    Stats target{
        10,
        20,
        30
    };

    const Stats source{
        2,
        3,
        4
    };

    simfw::sumStatsMembers(target, source);

    EXPECT_EQ(target.checks, 10u);
    EXPECT_EQ(target.candidates, 20u);
    EXPECT_EQ(target.unchanged, 30u);
}
