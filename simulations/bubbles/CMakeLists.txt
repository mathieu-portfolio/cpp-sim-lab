add_library(bubbles_core
    src/Simulation.cpp
    src/BubblePhysics.cpp
)

target_include_directories(bubbles_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(bubbles_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(bubbles_core
    PUBLIC
        cxx_std_20
)

add_executable(bubbles
    main.cpp
)

target_link_libraries(bubbles
    PRIVATE
        bubbles_core
        raylib
)
