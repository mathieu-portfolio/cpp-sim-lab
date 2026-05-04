add_executable(particles_tests
    collision_tests.cpp
    simulation_backend_tests.cpp
)

target_link_libraries(particles_tests
    PRIVATE
        particles_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(particles_tests)
