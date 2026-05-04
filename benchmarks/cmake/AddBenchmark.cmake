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

    string(REGEX REPLACE "_bench$" "" SIM_BENCH_ID "${SIM_BENCH_NAME}")
    set_property(
        GLOBAL
        APPEND
        PROPERTY CPP_SIM_BENCHMARK_REGISTRY_ENTRIES
        "${SIM_BENCH_ID}|${SIM_BENCH_NAME}|${CMAKE_CURRENT_SOURCE_DIR}"
    )
endfunction()

function(write_sim_benchmark_registry)
    get_property(
        SIM_BENCH_REGISTRY
        GLOBAL
        PROPERTY CPP_SIM_BENCHMARK_REGISTRY_ENTRIES
    )

    if(NOT SIM_BENCH_REGISTRY)
        message(STATUS "No benchmarks registered; skipping benchmark registry generation.")
        return()
    endif()

    list(JOIN SIM_BENCH_REGISTRY "\n" SIM_BENCH_REGISTRY_CONTENT)
    set(SIM_BENCH_REGISTRY_FILE "${CMAKE_BINARY_DIR}/benchmarks/registry.txt")
    get_filename_component(SIM_BENCH_REGISTRY_DIR "${SIM_BENCH_REGISTRY_FILE}" DIRECTORY)
    file(MAKE_DIRECTORY "${SIM_BENCH_REGISTRY_DIR}")
    file(WRITE "${SIM_BENCH_REGISTRY_FILE}" "${SIM_BENCH_REGISTRY_CONTENT}\n")

    message(STATUS "Wrote benchmark registry: ${SIM_BENCH_REGISTRY_FILE}")
endfunction()
