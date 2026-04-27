add_executable(particles_fixed_grid_compare_bench
    main.cpp
    ../../simulations/particles_cpu/SpatialGrid.cpp
)

target_include_directories(particles_fixed_grid_compare_bench
    PRIVATE
        ../../simulations/particles_cpu
)

target_link_libraries(particles_fixed_grid_compare_bench
    PRIVATE
        sim_framework
)

target_compile_features(particles_fixed_grid_compare_bench
    PRIVATE
        cxx_std_20
)