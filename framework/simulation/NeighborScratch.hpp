#pragma once

#include <cstddef>
#include <vector>

namespace simfw::simulation {

template <typename Index = std::size_t>
struct NeighborScratch {
    std::vector<Index> candidates;
    std::vector<Index> neighbors;
    std::vector<Index> secondaryNeighbors;
};

} // namespace simfw::simulation
