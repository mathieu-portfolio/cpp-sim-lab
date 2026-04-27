add_library(boids_cpu_core
    src/Simulation.cpp
)

target_include_directories(boids_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(boids_cpu_core
    PUBLIC
        sim_framework
)

target_compile_features(boids_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(boids_cpu
    main.cpp
)

target_link_libraries(boids_cpu
    PRIVATE
        boids_cpu_core
        raylib
)
