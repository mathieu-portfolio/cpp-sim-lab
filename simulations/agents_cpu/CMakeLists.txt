add_library(agents_cpu_core
    src/Simulation.cpp
)

target_include_directories(agents_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(agents_cpu_core
    PUBLIC
        sim_framework
        thread_pool
)

target_compile_features(agents_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(agents_cpu
    main.cpp
)

target_link_libraries(agents_cpu
    PRIVATE
        agents_cpu_core
        raylib
)
