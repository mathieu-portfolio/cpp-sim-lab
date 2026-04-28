#include <simulation/CollisionPairs.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <utility>
#include <vector>

namespace {
using Pair = std::pair<int, int>;
}

TEST(CollisionPairsTest, VisitsEachUniqueCandidatePairOnce) {
    std::vector<int> candidates;
    std::vector<Pair> visited;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        3,
        candidates,
        [](int source, std::vector<int>& out) {
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
    std::vector<int> candidates;
    std::vector<Pair> visited;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        4,
        candidates,
        [](int source, std::vector<int>& out) {
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
    std::vector<int> candidates;
    std::vector<Pair> visited;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        5,
        candidates,
        [](int, std::vector<int>& out) {
            out.clear();
        },
        [&visited](int a, int b) {
            visited.emplace_back(a, b);
        }
    );

    EXPECT_TRUE(visited.empty());
}

TEST(CollisionPairsTest, ReusesCallerProvidedCandidateStorage) {
    std::vector<int> candidates;
    candidates.reserve(16);
    const auto* initialData = candidates.data();

    std::size_t visitCount = 0;

    simfw::simulation::forEachUniqueCandidatePair<int>(
        2,
        candidates,
        [](int, std::vector<int>& out) {
            out = {0, 1};
        },
        [&visitCount](int, int) {
            ++visitCount;
        }
    );

    EXPECT_EQ(visitCount, 1U);
    EXPECT_EQ(candidates.capacity(), 16U);
    EXPECT_EQ(candidates.data(), initialData);
}
