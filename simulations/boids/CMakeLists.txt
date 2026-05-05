add_library(boids_core
    src/Simulation.cpp
    src/BoidBehavior.cpp
    src/BoidSteering.cpp
)

target_include_directories(boids_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(boids_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(boids_core
    PUBLIC
        cxx_std_20
)

add_executable(boids
    main.cpp
)

target_link_libraries(boids
    PRIVATE
        boids_core
        raylib
)
