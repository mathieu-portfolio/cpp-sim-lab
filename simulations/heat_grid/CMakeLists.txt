add_library(heat_grid_core
    src/Simulation.cpp
)

target_include_directories(heat_grid_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(heat_grid_core
    PUBLIC
        sim_framework
)

target_compile_features(heat_grid_core
    PUBLIC
        cxx_std_20
)

add_executable(heat_grid
    main.cpp
)

target_link_libraries(heat_grid
    PRIVATE
        heat_grid_core
        raylib
)
