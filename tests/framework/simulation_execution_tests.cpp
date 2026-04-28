#include <simulation/NeighborScratch.hpp>
#include <simulation/SimulationExecutionConfig.hpp>

#include <gtest/gtest.h>

#include <cstddef>

TEST(SimulationExecutionConfigTests, DefaultsToEnabledBackends) {
    const simfw::simulation::SimulationExecutionConfig config{};

    EXPECT_TRUE(config.useSpatialGrid);
    EXPECT_TRUE(config.useParallelUpdate);
}

TEST(SimulationExecutionConfigTests, SupportsExplicitBackendDefaults) {
    const simfw::simulation::SimulationExecutionConfig config{false, true};

    EXPECT_FALSE(config.useSpatialGrid);
    EXPECT_TRUE(config.useParallelUpdate);
}

TEST(NeighborScratchTests, ProvidesReusableCandidateAndNeighborBuffers) {
    simfw::simulation::NeighborScratch<std::size_t> scratch;

    scratch.candidates = {1, 2, 3};
    scratch.neighbors = {1};
    scratch.secondaryNeighbors = {2, 3};

    EXPECT_EQ(scratch.candidates.size(), 3u);
    EXPECT_EQ(scratch.neighbors.size(), 1u);
    EXPECT_EQ(scratch.secondaryNeighbors.size(), 2u);
}
