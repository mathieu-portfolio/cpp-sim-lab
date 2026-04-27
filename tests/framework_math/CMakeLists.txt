add_executable(framework_math_tests
    main.cpp
)

target_link_libraries(framework_math_tests
    PRIVATE
        sim_framework
        GTest::gtest_main
)

target_compile_features(framework_math_tests
    PRIVATE
        cxx_std_20
)

include(GoogleTest)
gtest_discover_tests(framework_math_tests)