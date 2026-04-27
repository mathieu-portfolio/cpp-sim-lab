function(add_sim_benchmark)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCES LIBS)

    cmake_parse_arguments(
        SIM_BENCH
        "${options}"
        "${oneValueArgs}"
        "${multiValueArgs}"
        ${ARGN}
    )

    if(NOT SIM_BENCH_NAME)
        message(FATAL_ERROR "add_sim_benchmark requires NAME")
    endif()

    if(NOT SIM_BENCH_SOURCES)
        message(FATAL_ERROR "add_sim_benchmark requires SOURCES")
    endif()

    add_executable(${SIM_BENCH_NAME}
        ${SIM_BENCH_SOURCES}
    )

    target_include_directories(${SIM_BENCH_NAME}
        PRIVATE
            ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../common
    )

    if(SIM_BENCH_LIBS)
        target_link_libraries(${SIM_BENCH_NAME}
            PRIVATE
                ${SIM_BENCH_LIBS}
        )
    endif()

    target_compile_features(${SIM_BENCH_NAME}
        PRIVATE
            cxx_std_20
    )
endfunction()
