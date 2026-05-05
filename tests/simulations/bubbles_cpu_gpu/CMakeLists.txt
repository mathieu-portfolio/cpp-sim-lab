add_executable(bubbles_cpu_gpu_tests
    bubble_physics_tests.cpp
)

target_link_libraries(bubbles_cpu_gpu_tests
    PRIVATE
        bubbles_cpu_gpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(bubbles_cpu_gpu_tests)
