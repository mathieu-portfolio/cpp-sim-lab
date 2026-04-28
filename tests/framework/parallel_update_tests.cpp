#include <simulation/ParallelUpdate.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace {

struct Scratch {
    std::vector<std::size_t> visited;
};

struct Stats {
    std::size_t itemCount = 0;
    std::size_t chunkCount = 0;
};

class InlineThreadPool {
public:
    template <typename Fn>
    void parallel_for(std::size_t itemCount, std::size_t grainSize, Fn&& fn) {
        std::size_t taskIndex = 0;

        for (std::size_t begin = 0; begin < itemCount; begin += grainSize) {
            const std::size_t end = std::min(begin + grainSize, itemCount);
            fn(begin, end, taskIndex);
            ++taskIndex;
        }
    }
};

} // namespace

TEST(ParallelUpdateTests, DoesNothingForEmptyWorkload) {
    InlineThreadPool pool;
    bool didWork = false;
    bool didReduce = false;

    simfw::runParallelUpdate<InlineThreadPool, Scratch, Stats>(
        &pool,
        0,
        4,
        true,
        [&didWork](std::size_t, std::size_t, Scratch&, Stats&) {
            didWork = true;
        },
        [&didReduce](const Stats&) {
            didReduce = true;
        }
    );

    EXPECT_FALSE(didWork);
    EXPECT_FALSE(didReduce);
}

TEST(ParallelUpdateTests, UsesSingleChunkWhenParallelIsDisabled) {
    InlineThreadPool pool;
    std::size_t reducedItems = 0;
    std::size_t reducedChunks = 0;

    simfw::runParallelUpdate<InlineThreadPool, Scratch, Stats>(
        &pool,
        10,
        3,
        false,
        [](std::size_t begin, std::size_t end, Scratch& scratch, Stats& stats) {
            for (std::size_t i = begin; i < end; ++i) {
                scratch.visited.push_back(i);
            }

            stats.itemCount += scratch.visited.size();
            ++stats.chunkCount;
        },
        [&reducedItems, &reducedChunks](const Stats& stats) {
            reducedItems += stats.itemCount;
            reducedChunks += stats.chunkCount;
        }
    );

    EXPECT_EQ(reducedItems, 10u);
    EXPECT_EQ(reducedChunks, 1u);
}

TEST(ParallelUpdateTests, ReducesOneStatsObjectPerParallelChunk) {
    InlineThreadPool pool;
    std::size_t reducedItems = 0;
    std::size_t reducedChunks = 0;

    simfw::runParallelUpdate<InlineThreadPool, Scratch, Stats>(
        &pool,
        10,
        4,
        true,
        [](std::size_t begin, std::size_t end, Scratch& scratch, Stats& stats) {
            for (std::size_t i = begin; i < end; ++i) {
                scratch.visited.push_back(i);
            }

            stats.itemCount += scratch.visited.size();
            ++stats.chunkCount;
        },
        [&reducedItems, &reducedChunks](const Stats& stats) {
            reducedItems += stats.itemCount;
            reducedChunks += stats.chunkCount;
        }
    );

    EXPECT_EQ(reducedItems, 10u);
    EXPECT_EQ(reducedChunks, 3u);
}
