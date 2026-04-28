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

} // namespace simfw::simulation
