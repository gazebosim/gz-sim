set(TEST_TYPE "BENCHMARK")

# generate a version_info.json file that can be used to
add_custom_target(version_target
  COMMAND ${CMAKE_COMMAND}
    -Dinput_file=${CMAKE_CURRENT_SOURCE_DIR}/version_info.json.in
    -Doutput_file=${CMAKE_CURRENT_BINARY_DIR}/version_info.json
    -Drepository_root=${CMAKE_CURRENT_SOURCE_DIR}
    -Dbuild_type=${CMAKE_BUILD_TYPE}
    -Dversion=${PROJECT_VERSION}
    -Dversion_full=${PROJECT_VERSION_FULL}
    -Dmajor=${PROJECT_VERSION_MAJOR}
    -Dminor=${PROJECT_VERSION_MINOR}
    -Dpatch=${PROJECT_VERSION_PATCH}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/generate_version.cmake
)

set(tests
  each.cc
  ecm_serialize.cc
)

ign_build_executables(
  PREFIX "${TEST_TYPE}_"
  SOURCES ${tests}
  LIB_DEPS gbenchmark gbenchmark_main
  EXEC_LIST test_list
)

foreach(test ${test_list})
  add_dependencies(${test} version_target)
endforeach()
