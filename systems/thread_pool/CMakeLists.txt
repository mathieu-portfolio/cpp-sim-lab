cmake_minimum_required(VERSION 3.16)

project(thread_pool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(THREAD_POOL_STANDALONE ON)
else()
    set(THREAD_POOL_STANDALONE OFF)
endif()

# -----------------------------
# Library
# -----------------------------
add_library(thread_pool
    src/thread_pool.cpp
)

target_include_directories(thread_pool
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(thread_pool
    PUBLIC
        cxx_std_17
)

# -----------------------------
# Demo
# -----------------------------
if(THREAD_POOL_STANDALONE)
    add_executable(thread_pool_demo
        demo/demo.cpp
    )

    target_link_libraries(thread_pool_demo
        PRIVATE thread_pool
    )
endif()

# -----------------------------
# Testing
# -----------------------------
include(CTest)

if(BUILD_TESTING)
    if(THREAD_POOL_STANDALONE)
        include(FetchContent)

        FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.17.0
        )

        FetchContent_MakeAvailable(googletest)
    endif()

    add_executable(thread_pool_tests
        tests/basic_tests.cpp
        tests/shutdown_tests.cpp
        tests/stress_tests.cpp
        tests/contract_tests.cpp
        tests/exception_tests.cpp
    )

    target_link_libraries(thread_pool_tests
        PRIVATE
            thread_pool
            GTest::gtest_main
    )

    target_include_directories(thread_pool_tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )

    include(GoogleTest)
    gtest_discover_tests(thread_pool_tests)
endif()

# -----------------------------
# Benchmark
# -----------------------------
if(THREAD_POOL_STANDALONE)
    add_executable(thread_pool_bench
        bench/bench.cpp
    )

    target_link_libraries(thread_pool_bench
        PRIVATE thread_pool
    )

    target_include_directories(thread_pool_bench
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()
