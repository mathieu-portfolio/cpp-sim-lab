#include <simulation/CollisionPairs.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace {
using Pair = std::pair<int, int>;
}

TEST(CollisionPairsTest, VisitsEachUniqueCandidatePairOnce) {
    std::vector<Pair> visited;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        3,
        [](std::size_t, std::vector<int>& out) {
            out = {0, 1, 2};
        },
        [&visited](int a, int b) {
            visited.emplace_back(a, b);
        }
    );

    const std::vector<Pair> expected{
        {0, 1},
        {0, 2},
        {1, 2}
    };

    EXPECT_EQ(visited, expected);
}

TEST(CollisionPairsTest, IgnoresSelfAndPreviouslyVisitedPairs) {
    std::vector<Pair> visited;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        4,
        [](std::size_t source, std::vector<int>& out) {
            switch (source) {
            case 0:
                out = {0, 1, 2};
                break;
            case 1:
                out = {0, 1, 3};
                break;
            case 2:
                out = {0, 2};
                break;
            case 3:
                out = {1, 3};
                break;
            default:
                out.clear();
                break;
            }
        },
        [&visited](int a, int b) {
            visited.emplace_back(a, b);
        }
    );

    const std::vector<Pair> expected{
        {0, 1},
        {0, 2},
        {1, 3}
    };

    EXPECT_EQ(visited, expected);
}

TEST(CollisionPairsTest, SupportsEmptyCandidateLists) {
    std::vector<Pair> visited;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        5,
        [](std::size_t, std::vector<int>& out) {
            out.clear();
        },
        [&visited](int a, int b) {
            visited.emplace_back(a, b);
        }
    );

    EXPECT_TRUE(visited.empty());
}

TEST(CollisionPairsTest, ReusesInternalCandidateStorageAcrossItems) {
    const int* firstData = nullptr;
    std::size_t firstCapacity = 0;
    std::size_t visitCount = 0;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        2,
        [&firstData, &firstCapacity](std::size_t source, std::vector<int>& out) {
            if (source == 0) {
                out.reserve(16);
                out = {0, 1};
                firstData = out.data();
                firstCapacity = out.capacity();
                return;
            }

            out.clear();
            EXPECT_EQ(out.capacity(), firstCapacity);
            EXPECT_EQ(out.data(), firstData);
        },
        [&visitCount](int, int) {
            ++visitCount;
        }
    );

    EXPECT_EQ(visitCount, 1U);
    EXPECT_EQ(firstCapacity, 16U);
}
