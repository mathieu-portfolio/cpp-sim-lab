#pragma once

#include <cstddef>
#include <vector>

namespace simfw::simulation {

// Iterates unique candidate pairs where the first index owns the query and the
// second index comes from a caller-provided candidate list. Pair mutation stays
// in the caller-provided lambda so this helper does not know about physics,
// particles, or collision resolution policy.
template <typename Index, typename CollectCandidatesFn, typename PairFn>
void forEachUniqueCandidatePair(
    std::size_t itemCount,
    CollectCandidatesFn&& collectCandidates,
    PairFn&& visitPair
) {
    std::vector<Index> candidates;

    for (std::size_t i = 0; i < itemCount; ++i) {
        collectCandidates(i, candidates);

        for (Index candidateIndex : candidates) {
            if (candidateIndex <= static_cast<Index>(i)) {
                continue;
            }

            visitPair(static_cast<Index>(i), candidateIndex);
        }
    }
}

// Compatibility overload for call sites that own and reuse candidate storage.
// The collector receives the current source index and the caller-provided
// candidate vector, preserving allocation behavior while sharing the same
// unique-pair filtering logic.
template <typename Index, typename CollectCandidatesFn, typename PairFn>
void forEachUniqueCandidatePair(
    std::size_t itemCount,
    std::vector<Index>& candidates,
    CollectCandidatesFn&& collectCandidates,
    PairFn&& visitPair
) {
    for (std::size_t i = 0; i < itemCount; ++i) {
        collectCandidates(static_cast<Index>(i), candidates);

        for (Index candidateIndex : candidates) {
            if (candidateIndex <= static_cast<Index>(i)) {
                continue;
            }

            visitPair(static_cast<Index>(i), candidateIndex);
        }
    }
}

} // namespace simfw::simulation
