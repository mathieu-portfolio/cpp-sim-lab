#pragma once

#include <cstdint>
#include <cstddef>

namespace bench {

inline std::uint32_t seedFor(std::uint32_t baseSeed, std::size_t value) {
    return baseSeed ^ static_cast<std::uint32_t>(
        value + 0x9e3779b9u + (baseSeed << 6u) + (baseSeed >> 2u)
    );
}

} // namespace bench
