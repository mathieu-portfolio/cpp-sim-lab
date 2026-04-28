#pragma once

namespace simfw {

template <typename StatsT, typename... MemberPtrs>
void sumStatsMembers(
    StatsT& target,
    const StatsT& source,
    MemberPtrs... members
) {
    ((target.*members += source.*members), ...);
}

} // namespace simfw
