add_executable(boids_cpu_tests
    boid_behavior_tests.cpp
    boid_simulation_tests.cpp
)

target_link_libraries(boids_cpu_tests
    PRIVATE
        boids_cpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(boids_cpu_tests)
