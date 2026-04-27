add_executable(particles_collision_tests
    main.cpp
    ../../simulations/particles_cpu/ParticlePhysics.cpp
)

target_include_directories(particles_collision_tests
    PRIVATE
        ../../simulations/particles_cpu
)

target_link_libraries(particles_collision_tests
    PRIVATE
        sim_framework
        GTest::gtest_main
)

target_compile_features(particles_collision_tests
    PRIVATE
        cxx_std_20
)

include(GoogleTest)
gtest_discover_tests(particles_collision_tests)