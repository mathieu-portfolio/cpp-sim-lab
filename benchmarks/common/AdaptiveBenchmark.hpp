#pragma once

#include <BenchTimer.hpp>

#include <algorithm>
#include <cstddef>

namespace bench {

struct AdaptiveFrameBudget {
    double targetMs = 1000.0;
    std::size_t minFrames = 30;
    std::size_t maxFrames = 600;
};

struct AdaptiveRunResult {
    double totalMs = 0.0;
    double avgFrameMs = 0.0;
    std::size_t frames = 0;
};

template <typename Fn>
AdaptiveRunResult runAdaptiveFrames(const AdaptiveFrameBudget& budget, Fn&& frameStep) {
    const std::size_t frameLimit = std::max(budget.minFrames, budget.maxFrames);

    const auto start = now();
    std::size_t frames = 0;
    while (frames < frameLimit) {
        std::forward<Fn>(frameStep)();
        ++frames;

        if (frames >= budget.minFrames && elapsedMs(start, now()) >= budget.targetMs) {
            break;
        }
    }

    const double totalMs = elapsedMs(start, now());
    const double avgFrameMs = frames > 0 ? totalMs / static_cast<double>(frames) : 0.0;
    return AdaptiveRunResult{totalMs, avgFrameMs, frames};
}

inline bool exceedsSlowThreshold(const AdaptiveRunResult& run, double slowFrameThresholdMs) {
    return run.avgFrameMs >= slowFrameThresholdMs;
}

} // namespace bench
