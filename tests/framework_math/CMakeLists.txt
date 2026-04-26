add_executable(framework_math_tests
    main.cpp
)

target_link_libraries(framework_math_tests
    PRIVATE
        sim_framework
)

target_compile_features(framework_math_tests
    PRIVATE
        cxx_std_20
)

add_test(NAME framework_math_tests COMMAND framework_math_tests)
