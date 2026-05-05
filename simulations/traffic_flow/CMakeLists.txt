add_library(traffic_flow_core
    src/Simulation.cpp
    src/CrossroadPolicy.cpp
    src/TrafficPhysics.cpp
    src/VehicleBehavior.cpp
    src/RoadTopologyServices.cpp
    src/TrafficLifecycle.cpp
)

target_include_directories(traffic_flow_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(traffic_flow_core
    PUBLIC
        sim_framework
)

target_compile_features(traffic_flow_core
    PUBLIC
        cxx_std_20
)

add_executable(traffic_flow
    main.cpp
)

target_link_libraries(traffic_flow
    PRIVATE
        traffic_flow_core
        raylib
)
