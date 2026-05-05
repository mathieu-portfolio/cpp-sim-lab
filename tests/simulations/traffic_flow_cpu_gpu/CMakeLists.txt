add_executable(traffic_flow_cpu_gpu_simulation_tests
    simulation_tests.cpp
)

target_link_libraries(traffic_flow_cpu_gpu_simulation_tests
    PRIVATE
        traffic_flow_cpu_gpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(traffic_flow_cpu_gpu_simulation_tests)
