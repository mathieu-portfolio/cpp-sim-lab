cmake_minimum_required(VERSION 3.16)

project(sim_framework LANGUAGES CXX)

add_library(sim_framework INTERFACE)

target_include_directories(sim_framework
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(sim_framework
    INTERFACE
        cxx_std_20
)