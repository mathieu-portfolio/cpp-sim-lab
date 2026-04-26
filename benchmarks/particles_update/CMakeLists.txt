add_executable(particles_update_bench
    main.cpp
)

target_link_libraries(particles_update_bench
    PRIVATE
        sim_framework
)

target_compile_features(particles_update_bench
    PRIVATE
        cxx_std_20
)
