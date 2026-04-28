cmake_minimum_required(VERSION 3.16)

project(cpp_sim_lab LANGUAGES CXX)

enable_testing()
include(CTest)

if(BUILD_TESTING)
    include(FetchContent)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.17.0
    )

    FetchContent_MakeAvailable(googletest)
endif()

find_package(raylib CONFIG REQUIRED)

add_subdirectory(framework)

add_subdirectory(simulations/particles_cpu)
add_subdirectory(simulations/boids_cpu)
add_subdirectory(simulations/agents_cpu)

add_subdirectory(tests/framework)
add_subdirectory(tests/simulations/particles_cpu)
add_subdirectory(tests/simulations/boids_cpu)

add_subdirectory(benchmarks)
