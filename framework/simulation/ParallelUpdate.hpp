#pragma once

#include <cstddef>
#include <utility>
#include <vector>

namespace simfw {

// Runs a chunked update over [0, itemCount), optionally using the supplied
// thread pool. Each chunk receives isolated scratch and stats objects, then
// the caller-provided reducer merges those stats after all work is complete.
//
// This keeps simulation-specific behavior in the caller while centralizing the
// common orchestration pattern shared by CPU simulations: grain sizing,
// per-worker temporary storage, and deterministic stats reduction.
template <typename ThreadPoolT, typename ScratchT, typename StatsT, typename WorkFn, typename ReduceFn>
void runParallelUpdate(
    ThreadPoolT* threadPool,
    std::size_t itemCount,
    std::size_t grainSize,
    bool useParallel,
    WorkFn&& work,
    ReduceFn&& reduce
) {
    if (itemCount == 0) {
        return;
    }

    const std::size_t safeGrainSize = grainSize == 0 ? itemCount : grainSize;
    const std::size_t taskCount =
        (itemCount + safeGrainSize - 1) / safeGrainSize;

    if (!useParallel || threadPool == nullptr || taskCount <= 1) {
        ScratchT scratch;
        StatsT stats{};

        work(0, itemCount, scratch, stats);
        reduce(stats);
        return;
    }

    std::vector<ScratchT> workerScratch(taskCount);
    std::vector<StatsT> workerStats(taskCount);

    threadPool->parallel_for(
        itemCount,
        safeGrainSize,
        [&work, &workerScratch, &workerStats](
            std::size_t beginIndex,
            std::size_t endIndex,
            std::size_t taskIndex
        ) {
            work(
                beginIndex,
                endIndex,
                workerScratch[taskIndex],
                workerStats[taskIndex]
            );
        }
    );

    for (const StatsT& stats : workerStats) {
        reduce(stats);
    }
}

} // namespace simfw
