add_library(predator_prey_cpu_core
    src/Simulation.cpp
)

target_include_directories(predator_prey_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(predator_prey_cpu_core
    PUBLIC
        sim_framework
)

target_compile_features(predator_prey_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(predator_prey_cpu
    main.cpp
)

target_link_libraries(predator_prey_cpu
    PRIVATE
        predator_prey_cpu_core
        raylib
)
