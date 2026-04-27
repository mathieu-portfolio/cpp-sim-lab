add_executable(particles_collision_tests
    main.cpp
)

target_link_libraries(particles_collision_tests
    PRIVATE
        particles_cpu_core
        GTest::gtest_main
)

target_compile_features(particles_collision_tests
    PRIVATE
        cxx_std_20
)

include(GoogleTest)
gtest_discover_tests(particles_collision_tests)