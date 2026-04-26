cmake_minimum_required(VERSION 3.16)

project(cpp_sim_lab LANGUAGES CXX)

enable_testing()

find_package(raylib CONFIG REQUIRED)

add_subdirectory(framework)
add_subdirectory(simulations/particles_cpu)
add_subdirectory(tests/framework_math)
add_subdirectory(benchmarks/particles_update)
add_subdirectory(benchmarks/particles_collision)
