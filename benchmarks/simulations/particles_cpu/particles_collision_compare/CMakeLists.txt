add_executable(particles_collision_compare_bench
    main.cpp
)

target_link_libraries(particles_collision_compare_bench
    PRIVATE
        particles_cpu_core
)

target_compile_features(particles_collision_compare_bench
    PRIVATE
        cxx_std_20
)
