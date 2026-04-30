#pragma once

#include <cstddef>
#include <random/Random.hpp>

namespace simfw::simulation {

inline Vec2 randomDiscOffset(float radius) {
    for (;;) {
        const Vec2 offset{
            Random::range(-radius, radius),
            Random::range(-radius, radius)
        };

        if (offset.lengthSquared() <= radius * radius) {
            return offset;
        }
    }
}

template <typename SpawnFn>
std::size_t spawnBrush(
    std::size_t currentCount,
    std::size_t maxCount,
    int spawnCount,
    SpawnFn&& spawnEntity
) {
    std::size_t spawned = 0;

    for (int i = 0; i < spawnCount && currentCount + spawned < maxCount; ++i) {
        spawnEntity();
        ++spawned;
    }

    return spawned;
}

} // namespace simfw::simulation
