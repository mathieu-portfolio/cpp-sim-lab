add_executable(boids_naive_neighbors_bench
    main.cpp
)

target_link_libraries(boids_naive_neighbors_bench
    PRIVATE
        boids_cpu_core
)