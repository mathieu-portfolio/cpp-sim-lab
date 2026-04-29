add_executable(bubbles_cpu_tests
    bubble_physics_tests.cpp
)

target_link_libraries(bubbles_cpu_tests
    PRIVATE
        bubbles_cpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(bubbles_cpu_tests)
