add_executable(framework_tests
    math_tests.cpp
    parallel_update_tests.cpp
    spatial_query_tests.cpp
    stats_reduction_tests.cpp
)

target_link_libraries(framework_tests
    PRIVATE
        sim_framework
        GTest::gtest_main
)

target_compile_features(framework_tests
    PRIVATE
        cxx_std_20
)

include(GoogleTest)
gtest_discover_tests(framework_tests)
