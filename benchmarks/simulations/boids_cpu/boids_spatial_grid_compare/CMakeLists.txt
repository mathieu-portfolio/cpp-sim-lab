add_executable(boids_spatial_grid_compare_bench
    main.cpp
)

target_link_libraries(boids_spatial_grid_compare_bench
    PRIVATE
        boids_cpu_core
)

target_compile_features(boids_spatial_grid_compare_bench
    PRIVATE
        cxx_std_20
)
