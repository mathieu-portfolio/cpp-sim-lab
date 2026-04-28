#pragma once

#include <math/Vec2.hpp>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace simfw::simulation {

enum class SpatialQueryBackend {
    Naive,
    Grid
};

template <typename Index = std::size_t>
struct SpatialQueryOptions {
    SpatialQueryBackend backend = SpatialQueryBackend::Grid;
    std::size_t itemCount = 0;
    bool excludeIndex = false;
    Index excludedIndex = std::numeric_limits<Index>::max();
};

template <typename Index = std::size_t>
SpatialQueryOptions<Index> makeSpatialQueryOptions(
    bool useSpatialGrid,
    std::size_t itemCount
) {
    return SpatialQueryOptions<Index>{
        useSpatialGrid ? SpatialQueryBackend::Grid : SpatialQueryBackend::Naive,
        itemCount,
        false,
        std::numeric_limits<Index>::max()
    };
}

template <typename Index = std::size_t>
SpatialQueryOptions<Index> makeSpatialQueryOptionsExcluding(
    bool useSpatialGrid,
    std::size_t itemCount,
    Index excludedIndex
) {
    return SpatialQueryOptions<Index>{
        useSpatialGrid ? SpatialQueryBackend::Grid : SpatialQueryBackend::Naive,
        itemCount,
        true,
        excludedIndex
    };
}

template <typename Index>
void collectNaiveCandidates(
    const SpatialQueryOptions<Index>& options,
    std::vector<Index>& out
) {
    out.clear();
    out.reserve(options.itemCount);

    for (std::size_t i = 0; i < options.itemCount; ++i) {
        const Index index = static_cast<Index>(i);

        if (options.excludeIndex && index == options.excludedIndex) {
            continue;
        }

        out.push_back(index);
    }
}

template <typename Index>
void filterExcludedIndex(
    const SpatialQueryOptions<Index>& options,
    std::vector<Index>& out
) {
    if (!options.excludeIndex) {
        return;
    }

    out.erase(
        std::remove(out.begin(), out.end(), options.excludedIndex),
        out.end()
    );
}

template <typename Grid, typename Index = std::size_t>
void collectCandidates(
    const Grid& grid,
    Vec2 position,
    float radius,
    const SpatialQueryOptions<Index>& options,
    std::vector<Index>& out
) {
    switch (options.backend) {
    case SpatialQueryBackend::Naive:
        collectNaiveCandidates(options, out);
        return;
    case SpatialQueryBackend::Grid:
        out.clear();
        grid.queryRadius(position, radius, out);
        filterExcludedIndex(options, out);
        return;
    }

    collectNaiveCandidates(options, out);
}

} // namespace simfw::simulation
