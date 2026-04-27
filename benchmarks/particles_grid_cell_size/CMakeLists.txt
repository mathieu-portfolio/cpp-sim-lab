add_executable(particles_grid_cell_size_bench
    main.cpp
)

target_link_libraries(particles_grid_cell_size_bench
    PRIVATE
        particles_cpu_core
)

target_compile_features(particles_grid_cell_size_bench
    PRIVATE
        cxx_std_20
)