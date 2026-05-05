add_executable(crowd_flow_field_tests
    flow_field_tests.cpp
)

target_link_libraries(crowd_flow_field_tests
    PRIVATE
        crowd_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(crowd_flow_field_tests)
