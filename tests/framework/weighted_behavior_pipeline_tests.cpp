#include <simulation/WeightedBehaviorPipeline.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <span>
#include <vector>

namespace {
struct TestOutput {
    float value = 0.0f;

    TestOutput& operator+=(TestOutput other) {
        value += other.value;
        return *this;
    }
};

TestOutput operator*(TestOutput output, float scale) {
    return TestOutput{output.value * scale};
}

struct TestContext {
    std::vector<int> calls;
    bool allowFilteredBehavior = true;
    float globalScale = 1.0f;
};

struct TestBehavior {
    using ComputeFn = TestOutput (*)(std::size_t, TestContext&);

    ComputeFn compute = nullptr;
    float weight = 1.0f;
    bool enabled = true;
    int id = 0;
};

TestOutput behaviorOne(std::size_t entityIndex, TestContext& context) {
    context.calls.push_back(1);
    return TestOutput{static_cast<float>(entityIndex) + 1.0f};
}

TestOutput behaviorTwo(std::size_t, TestContext& context) {
    context.calls.push_back(2);
    return TestOutput{2.0f};
}

TestOutput behaviorThree(std::size_t, TestContext& context) {
    context.calls.push_back(3);
    return TestOutput{3.0f};
}

float weightFor(const TestBehavior& behavior, const TestContext&) {
    return behavior.weight;
}

bool appliesToContext(const TestBehavior& behavior, const TestContext& context) {
    return behavior.id != 2 || context.allowFilteredBehavior;
}

float scaleFor(const TestBehavior&, const TestContext& context) {
    return context.globalScale;
}
} // namespace

TEST(WeightedBehaviorPipelineTests, AppliesWeightsAndAccumulatesContributions) {
    std::array<TestBehavior, 2> behaviors{{
        TestBehavior{behaviorOne, 2.0f, true, 1},
        TestBehavior{behaviorTwo, 3.0f, true, 2}
    }};

    TestContext context;

    const TestOutput result =
        simfw::simulation::computeWeightedBehaviors<TestBehavior, TestContext, TestOutput>(
            4,
            std::span<const TestBehavior>{behaviors},
            context,
            weightFor
        );

    EXPECT_FLOAT_EQ(result.value, 16.0f);
    EXPECT_EQ(context.calls, (std::vector<int>{1, 2}));
}

TEST(WeightedBehaviorPipelineTests, SkipsDisabledAndNullBehaviors) {
    std::array<TestBehavior, 4> behaviors{{
        TestBehavior{behaviorOne, 1.0f, false, 1},
        TestBehavior{nullptr, 10.0f, true, 99},
        TestBehavior{behaviorTwo, 2.0f, true, 2},
        TestBehavior{behaviorThree, 3.0f, false, 3}
    }};

    TestContext context;

    const TestOutput result =
        simfw::simulation::computeWeightedBehaviors<TestBehavior, TestContext, TestOutput>(
            0,
            std::span<const TestBehavior>{behaviors},
            context,
            weightFor
        );

    EXPECT_FLOAT_EQ(result.value, 4.0f);
    EXPECT_EQ(context.calls, (std::vector<int>{2}));
}

TEST(WeightedBehaviorPipelineTests, PreservesBehaviorOrder) {
    std::array<TestBehavior, 3> behaviors{{
        TestBehavior{behaviorThree, 1.0f, true, 3},
        TestBehavior{behaviorOne, 1.0f, true, 1},
        TestBehavior{behaviorTwo, 1.0f, true, 2}
    }};

    TestContext context;

    static_cast<void>(
        simfw::simulation::computeWeightedBehaviors<TestBehavior, TestContext, TestOutput>(
            0,
            std::span<const TestBehavior>{behaviors},
            context,
            weightFor
        )
    );

    EXPECT_EQ(context.calls, (std::vector<int>{3, 1, 2}));
}

TEST(WeightedBehaviorPipelineTests, AppliesContextFilterAndScale) {
    std::array<TestBehavior, 3> behaviors{{
        TestBehavior{behaviorOne, 1.0f, true, 1},
        TestBehavior{behaviorTwo, 100.0f, true, 2},
        TestBehavior{behaviorThree, 2.0f, true, 3}
    }};

    TestContext context;
    context.allowFilteredBehavior = false;
    context.globalScale = 0.5f;

    const TestOutput result =
        simfw::simulation::computeWeightedBehaviors<TestBehavior, TestContext, TestOutput>(
            1,
            std::span<const TestBehavior>{behaviors},
            context,
            weightFor,
            appliesToContext,
            scaleFor
        );

    EXPECT_FLOAT_EQ(result.value, 4.0f);
    EXPECT_EQ(context.calls, (std::vector<int>{1, 3}));
}
