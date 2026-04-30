cmake_minimum_required(VERSION 3.16)

project(cpp_sim_lab LANGUAGES CXX)

add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/EHsc>)

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
add_subdirectory(systems/thread_pool)
add_subdirectory(simulations)
add_subdirectory(tests)
add_subdirectory(benchmarks)
