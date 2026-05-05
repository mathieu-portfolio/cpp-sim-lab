add_executable(bubbles_tests
    bubble_physics_tests.cpp
)

target_link_libraries(bubbles_tests
    PRIVATE
        bubbles_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(bubbles_tests)
