add_library(traffic_flow_cpu_core
    src/Simulation.cpp
)

target_include_directories(traffic_flow_cpu_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(traffic_flow_cpu_core
    PUBLIC
        sim_framework
)

target_compile_features(traffic_flow_cpu_core
    PUBLIC
        cxx_std_20
)

add_executable(traffic_flow_cpu
    main.cpp
)

target_link_libraries(traffic_flow_cpu
    PRIVATE
        traffic_flow_cpu_core
        raylib
)
