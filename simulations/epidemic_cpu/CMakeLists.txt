add_library(epidemic_cpu_core
    src/Simulation.cpp
)

target_include_directories(epidemic_cpu_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(epidemic_cpu_core PUBLIC sim_framework)
target_compile_features(epidemic_cpu_core PUBLIC cxx_std_20)

add_executable(epidemic_cpu main.cpp)
target_link_libraries(epidemic_cpu PRIVATE epidemic_cpu_core raylib)
