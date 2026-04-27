add_executable(particles_fixed_grid_compare_bench
    main.cpp
)

target_link_libraries(particles_fixed_grid_compare_bench
    PRIVATE
      particles_cpu_core
)

target_compile_features(particles_fixed_grid_compare_bench
    PRIVATE
        cxx_std_20
)