add_executable(boids_tests
    boid_behavior_tests.cpp
    boid_simulation_tests.cpp
)

target_link_libraries(boids_tests
    PRIVATE
        boids_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(boids_tests)
