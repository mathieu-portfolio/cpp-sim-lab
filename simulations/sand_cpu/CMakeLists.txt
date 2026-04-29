add_library(sand_cpu_core
    src/Simulation.cpp
)

target_include_directories(sand_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sand_cpu_core
    PUBLIC
        sim_framework
)

target_compile_features(sand_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(sand_cpu
    main.cpp
)

target_link_libraries(sand_cpu
    PRIVATE
        sand_cpu_core
        raylib
)
