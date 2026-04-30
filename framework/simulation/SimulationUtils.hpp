#pragma once

#include <cstddef>
#include <thread>
#include <utility>

namespace simfw::simulation {

inline std::size_t hardwareWorkerCount() {
    const unsigned int hardwareThreads = std::thread::hardware_concurrency();

    if (hardwareThreads == 0) {
        return 1;
    }

    return static_cast<std::size_t>(hardwareThreads);
}

template <typename Config>
inline void syncEntityCount(std::size_t defaultCount, std::size_t& primaryCount, Config& config) {
    if (primaryCount == defaultCount && config.entityCount != defaultCount) {
        primaryCount = config.entityCount;
    } else {
        config.entityCount = primaryCount;
    }
}

template <typename Grid, typename Entities, typename PositionFn>
inline void setupSpatialIndex(
    Grid& grid,
    float cellSize,
    bool useSpatialGrid,
    const Entities& entities,
    PositionFn&& positionFn
) {
    grid.setCellSize(cellSize);

    if (useSpatialGrid) {
        grid.build(entities, std::forward<PositionFn>(positionFn));
    } else {
        grid.clear();
    }
}

template <typename Stats>
inline void syncEntityStatCount(Stats& stats, std::size_t count) {
    stats.entityCount = count;
}

} // namespace simfw::simulation
