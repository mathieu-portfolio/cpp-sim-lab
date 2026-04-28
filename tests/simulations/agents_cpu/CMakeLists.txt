add_executable(agents_cpu_tests
    # TODO: add tests here
)

target_link_libraries(agents_cpu_tests
    PRIVATE
        agents_cpu_core
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(agents_cpu_tests)
