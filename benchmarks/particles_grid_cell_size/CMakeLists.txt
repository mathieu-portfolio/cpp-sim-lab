add_executable(particles_grid_cell_size_bench
    main.cpp
    ../../simulations/particles_cpu/SpatialGrid.cpp
)

target_include_directories(particles_grid_cell_size_bench
    PRIVATE
        ../../simulations/particles_cpu
)

target_link_libraries(particles_grid_cell_size_bench
    PRIVATE
        sim_framework
)

target_compile_features(particles_grid_cell_size_bench
    PRIVATE
        cxx_std_20
)