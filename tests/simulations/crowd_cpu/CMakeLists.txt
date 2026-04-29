add_executable(crowd_cpu_flow_field_tests
    flow_field_tests.cpp
)

target_link_libraries(crowd_cpu_flow_field_tests
    PRIVATE
        crowd_cpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(crowd_cpu_flow_field_tests)
