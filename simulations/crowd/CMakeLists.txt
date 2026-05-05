add_library(crowd_core
    src/Simulation.cpp
    src/CrowdBehavior.cpp
)

target_include_directories(crowd_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(crowd_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(crowd_core
    PUBLIC
        cxx_std_20
)

add_executable(crowd
    main.cpp
)

target_link_libraries(crowd
    PRIVATE
        crowd_core
        raylib
)
