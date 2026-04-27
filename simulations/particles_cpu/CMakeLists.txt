add_executable(particles_cpu
    main.cpp
    Simulation.cpp
    SpatialGrid.cpp
    ParticlePhysics.cpp
)

target_include_directories(particles_cpu
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(particles_cpu
    PRIVATE
        sim_framework
        raylib
)

target_compile_features(particles_cpu
    PRIVATE
        cxx_std_20
)
