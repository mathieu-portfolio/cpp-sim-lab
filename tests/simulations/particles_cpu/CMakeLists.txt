add_executable(particles_cpu_tests
    collision_tests.cpp
    simulation_backend_tests.cpp
)

target_link_libraries(particles_cpu_tests
    PRIVATE
        particles_cpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(particles_cpu_tests)