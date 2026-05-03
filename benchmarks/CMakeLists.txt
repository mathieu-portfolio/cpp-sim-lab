include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddBenchmark.cmake)

add_subdirectory(simulations/particles_cpu)
add_subdirectory(simulations/boids_cpu)
add_subdirectory(simulations/agents_cpu)
add_subdirectory(simulations/crowd_cpu)

add_subdirectory(simulations/bubbles_cpu)
add_subdirectory(simulations/sand_cpu)
add_subdirectory(simulations/traffic_flow_cpu)
