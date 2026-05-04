add_library(particles_core
    src/Simulation.cpp
    src/ParticlePhysics.cpp
)

target_include_directories(particles_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(particles_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(particles_core
    PUBLIC
        cxx_std_20
)

add_executable(particles
    main.cpp
)

target_link_libraries(particles
    PRIVATE
        particles_core
        raylib
)
