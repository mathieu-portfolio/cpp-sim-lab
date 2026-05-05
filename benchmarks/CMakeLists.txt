include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddBenchmark.cmake)

add_subdirectory(simulations/particles)
add_subdirectory(simulations/boids)
add_subdirectory(simulations/agents_cpu)
add_subdirectory(simulations/crowd)

add_subdirectory(simulations/bubbles)
add_subdirectory(simulations/sand_cpu)

write_sim_benchmark_registry()

add_subdirectory(simulations/epidemic_cpu)
