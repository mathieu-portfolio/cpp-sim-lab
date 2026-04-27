#pragma once

#include <chrono>
#include <utility>

namespace bench {

using Clock = std::chrono::steady_clock;

inline Clock::time_point now() {
    return Clock::now();
}

inline double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

template <typename Fn>
double measureMs(Fn&& fn) {
    const auto start = now();
    std::forward<Fn>(fn)();
    const auto end = now();

    return elapsedMs(start, end);
}

} // namespace bench
