add_library(particles_cpu_core
    src/Simulation.cpp
    src/ParticlePhysics.cpp
)

target_include_directories(particles_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(particles_cpu_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(particles_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(particles_cpu
    main.cpp
)

target_link_libraries(particles_cpu
    PRIVATE
        particles_cpu_core
        raylib
)
