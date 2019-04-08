set(TEST_TYPE "BENCHMARK")

set(tests
  each.cc
  ecm_serialize.cc
)

ign_build_executables(
  PREFIX "${TEST_TYPE}_"
  SOURCES ${tests}
  LIB_DEPS gbenchmark gbenchmark_main
  EXEC_LIST benchmark_list
)

set(VERSION_FILE ${CMAKE_CURRENT_BINARY_DIR}/version_info.json)

# generate a version_info.json file that can be used to embed project
# version information
# While this command may look a bit unweildy, it creates a target
# that forces the file to be regenerated at build time.
add_custom_target(version_target
  COMMAND ${CMAKE_COMMAND}
    -Dinput_file=${CMAKE_CURRENT_SOURCE_DIR}/version_info.json.in
    -Doutput_file=${VERSION_FILE}
    -Drepository_root=${CMAKE_CURRENT_SOURCE_DIR}
    -Dbuild_type=${CMAKE_BUILD_TYPE}
    -Dversion=${PROJECT_VERSION}
    -Dversion_full=${PROJECT_VERSION_FULL}
    -Dmajor=${PROJECT_VERSION_MAJOR}
    -Dminor=${PROJECT_VERSION_MINOR}
    -Dpatch=${PROJECT_VERSION_PATCH}
    -Dproject_name=${PROJECT_NAME}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/generate_version.cmake
)

## Generate the run_benchmarks target
set(RESULTS_ROOT ${CMAKE_BINARY_DIR}/benchmark_results)
set(BENCHMARK_TARGETS "")
foreach(benchmark ${benchmark_list})
  list(APPEND BENCHMARK_TARGETS "$<TARGET_FILE:${benchmark}>")
endforeach()

# This has to be done in two phases:
# At configure time, the `@` fields are populated
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/run_benchmarks.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/run_benchmarks.py.gen
)

# At build time, the generator expressions are expanded.
file(
  GENERATE
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/run_benchmarks.py
  INPUT ${CMAKE_CURRENT_BINARY_DIR}/run_benchmarks.py.gen
)

add_custom_target(
  run_benchmarks
  COMMAND python3 ${CMAKE_CURRENT_BINARY_DIR}/run_benchmarks.py
)

# Make the run_benchmarks target dependent on the benchmarks and version
add_dependencies(run_benchmarks version_target ${benchmark_list})
