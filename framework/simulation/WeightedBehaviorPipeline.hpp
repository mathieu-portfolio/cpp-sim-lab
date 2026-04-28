#pragma once

#include <cstddef>
#include <span>

namespace simfw::simulation {

struct AlwaysApplyBehavior {
    template <typename Behavior, typename Context>
    constexpr bool operator()(const Behavior&, const Context&) const {
        return true;
    }
};

struct UnitBehaviorScale {
    template <typename Behavior, typename Context>
    constexpr float operator()(const Behavior&, const Context&) const {
        return 1.0f;
    }
};

template <typename Behavior, typename Context, typename Output, typename WeightFn, typename AppliesFn, typename ScaleFn>
Output computeWeightedBehaviors(
    std::size_t entityIndex,
    std::span<const Behavior> behaviors,
    Context& context,
    WeightFn weightFor,
    AppliesFn appliesToContext,
    ScaleFn scaleFor
) {
    Output result{};

    for (const Behavior& behavior : behaviors) {
        if (!behavior.enabled || behavior.compute == nullptr) {
            continue;
        }

        if (!appliesToContext(behavior, context)) {
            continue;
        }

        const Output contribution = behavior.compute(entityIndex, context);
        result += contribution * (weightFor(behavior, context) * scaleFor(behavior, context));
    }

    return result;
}

template <typename Behavior, typename Context, typename Output, typename WeightFn>
Output computeWeightedBehaviors(
    std::size_t entityIndex,
    std::span<const Behavior> behaviors,
    Context& context,
    WeightFn weightFor
) {
    return computeWeightedBehaviors<Behavior, Context, Output>(
        entityIndex,
        behaviors,
        context,
        weightFor,
        AlwaysApplyBehavior{},
        UnitBehaviorScale{}
    );
}

} // namespace simfw::simulation
