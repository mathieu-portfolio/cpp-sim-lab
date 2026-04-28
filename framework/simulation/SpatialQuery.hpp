#pragma once

#include <math/Vec2.hpp>

#include <cstddef>
#include <optional>
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
    std::optional<Index> excludedIndex;
};

template <typename Index>
void collectNaiveCandidates(
    std::size_t itemCount,
    std::optional<Index> excludedIndex,
    std::vector<Index>& out
) {
    out.clear();
    out.reserve(itemCount);

    for (std::size_t i = 0; i < itemCount; ++i) {
        const Index index = static_cast<Index>(i);

        if (excludedIndex && index == *excludedIndex) {
            continue;
        }

        out.push_back(index);
    }
}

template <typename Index>
void filterExcludedIndex(
    std::optional<Index> excludedIndex,
    std::vector<Index>& candidates
) {
    if (!excludedIndex) {
        return;
    }

    const Index excluded = *excludedIndex;

    auto write = candidates.begin();

    for (auto read = candidates.begin(); read != candidates.end(); ++read) {
        if (*read != excluded) {
            *write = *read;
            ++write;
        }
    }

    candidates.erase(write, candidates.end());
}

template <typename Grid, typename Index = std::size_t>
void collectSpatialCandidates(
    const Grid& grid,
    Vec2 position,
    float radius,
    std::optional<Index> excludedIndex,
    std::vector<Index>& out
) {
    out.clear();
    grid.queryRadius(position, radius, out);
    filterExcludedIndex(excludedIndex, out);
}

template <typename Grid, typename Index = std::size_t>
void collectCandidates(
    const Grid& grid,
    Vec2 position,
    float radius,
    SpatialQueryOptions<Index> options,
    std::vector<Index>& out
) {
    switch (options.backend) {
    case SpatialQueryBackend::Naive:
        collectNaiveCandidates(options.itemCount, options.excludedIndex, out);
        return;
    case SpatialQueryBackend::Grid:
        collectSpatialCandidates(grid, position, radius, options.excludedIndex, out);
        return;
    }

    collectNaiveCandidates(options.itemCount, options.excludedIndex, out);
}

template <typename Index = std::size_t>
SpatialQueryOptions<Index> makeSpatialQueryOptions(
    bool useSpatialGrid,
    std::size_t itemCount,
    std::optional<Index> excludedIndex = std::nullopt
) {
    return SpatialQueryOptions<Index>{
        useSpatialGrid ? SpatialQueryBackend::Grid : SpatialQueryBackend::Naive,
        itemCount,
        excludedIndex
    };
}

} // namespace simfw::simulation
