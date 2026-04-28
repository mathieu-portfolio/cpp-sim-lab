#pragma once

#include <cstddef>
#include <vector>

namespace simfw::simulation {

// Iterates unique unordered pairs produced from a per-item candidate query.
//
// The caller owns candidate collection and pair mutation. This helper only
// centralizes the common pair traversal rule used by collision simulations:
// query candidates for each item, skip already-visited pairs, and visit each
// remaining pair once as (lowerIndex, higherIndex).
template <typename Index = std::size_t, typename CollectCandidatesFn, typename VisitPairFn>
void forEachUniqueCandidatePair(
    std::size_t itemCount,
    std::vector<Index>& candidates,
    CollectCandidatesFn&& collectCandidates,
    VisitPairFn&& visitPair
) {
    for (std::size_t source = 0; source < itemCount; ++source) {
        const Index sourceIndex = static_cast<Index>(source);

        candidates.clear();
        collectCandidates(sourceIndex, candidates);

        for (Index candidateIndex : candidates) {
            if (candidateIndex <= sourceIndex) {
                continue;
            }

            visitPair(sourceIndex, candidateIndex);
        }
    }
}

} // namespace simfw::simulation
