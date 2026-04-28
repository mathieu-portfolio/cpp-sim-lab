#include <gtest/gtest.h>

#include <simulation/SpatialQuery.hpp>
#include <spatial/SpatialHashGrid.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace {

struct Item {
    Vec2 position;
};

std::vector<std::size_t> sorted(std::vector<std::size_t> values) {
    std::sort(values.begin(), values.end());
    return values;
}

} // namespace

TEST(SpatialQueryTests, NaiveBackendReturnsAllIndices) {
    std::vector<std::size_t> candidates;
    simfw::SpatialHashGrid<std::size_t> grid{10.0f};

    simfw::simulation::collectCandidates(
        grid,
        Vec2{},
        10.0f,
        simfw::simulation::SpatialQueryOptions<std::size_t>{
            simfw::simulation::SpatialQueryBackend::Naive,
            4
        },
        candidates
    );

    EXPECT_EQ(candidates, (std::vector<std::size_t>{0, 1, 2, 3}));
}

TEST(SpatialQueryTests, NaiveBackendCanExcludeSelf) {
    std::vector<std::size_t> candidates;
    simfw::SpatialHashGrid<std::size_t> grid{10.0f};

    simfw::simulation::collectCandidates(
        grid,
        Vec2{},
        10.0f,
        simfw::simulation::makeSpatialQueryOptionsExcluding(false, 5, std::size_t{2}),
        candidates
    );

    EXPECT_EQ(candidates, (std::vector<std::size_t>{0, 1, 3, 4}));
}

TEST(SpatialQueryTests, GridBackendQueriesNearbyCells) {
    const std::vector<Item> items{
        Item{Vec2{0.0f, 0.0f}},
        Item{Vec2{8.0f, 0.0f}},
        Item{Vec2{100.0f, 100.0f}}
    };

    simfw::SpatialHashGrid<std::size_t> grid{10.0f};
    grid.build(items, [](const Item& item) {
        return item.position;
    });

    std::vector<std::size_t> candidates;
    simfw::simulation::collectCandidates(
        grid,
        Vec2{0.0f, 0.0f},
        10.0f,
        simfw::simulation::makeSpatialQueryOptions(true, items.size()),
        candidates
    );

    EXPECT_EQ(sorted(candidates), (std::vector<std::size_t>{0, 1}));
}

TEST(SpatialQueryTests, GridBackendCanExcludeSelf) {
    const std::vector<Item> items{
        Item{Vec2{0.0f, 0.0f}},
        Item{Vec2{8.0f, 0.0f}}
    };

    simfw::SpatialHashGrid<std::size_t> grid{10.0f};
    grid.build(items, [](const Item& item) {
        return item.position;
    });

    std::vector<std::size_t> candidates;
    simfw::simulation::collectCandidates(
        grid,
        Vec2{0.0f, 0.0f},
        10.0f,
        simfw::simulation::makeSpatialQueryOptionsExcluding(true, items.size(), std::size_t{0}),
        candidates
    );

    EXPECT_EQ(candidates, (std::vector<std::size_t>{1}));
}

TEST(SpatialQueryTests, FactoryChoosesNaiveBackendWhenGridIsDisabled) {
    const auto options = simfw::simulation::makeSpatialQueryOptions(false, 7);

    EXPECT_EQ(options.backend, simfw::simulation::SpatialQueryBackend::Naive);
    EXPECT_EQ(options.itemCount, 7u);
    EXPECT_FALSE(options.excludeIndex);
}
