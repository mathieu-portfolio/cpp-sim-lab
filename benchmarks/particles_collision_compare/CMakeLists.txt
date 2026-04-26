add_executable(particles_collision_compare_bench
    main.cpp
    ../../simulations/particles_cpu/SpatialGrid.cpp
)

target_include_directories(particles_collision_compare_bench
    PRIVATE
        ../../simulations/particles_cpu
)

target_link_libraries(particles_collision_compare_bench
    PRIVATE
        sim_framework
)

target_compile_features(particles_collision_compare_bench
    PRIVATE
        cxx_std_20
)
