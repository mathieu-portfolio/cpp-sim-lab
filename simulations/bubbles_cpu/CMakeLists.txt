add_library(bubbles_cpu_core
    src/Simulation.cpp
    src/BubblePhysics.cpp
)

target_include_directories(bubbles_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(bubbles_cpu_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(bubbles_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(bubbles_cpu
    main.cpp
)

target_link_libraries(bubbles_cpu
    PRIVATE
        bubbles_cpu_core
        raylib
)
