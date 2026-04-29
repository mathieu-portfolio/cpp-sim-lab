add_library(crowd_cpu_core
    src/Simulation.cpp
    src/CrowdBehavior.cpp
)

target_include_directories(crowd_cpu_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(crowd_cpu_core PUBLIC sim_framework thread_pool)
target_compile_features(crowd_cpu_core PUBLIC cxx_std_20)

add_executable(crowd_cpu main.cpp)
target_link_libraries(crowd_cpu PRIVATE crowd_cpu_core raylib)
