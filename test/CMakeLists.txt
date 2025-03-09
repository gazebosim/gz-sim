configure_file (test_config.hh.in
  ${PROJECT_BINARY_DIR}/include/test_config.hh
)
include_directories (
  ${PROJECT_BINARY_DIR}/include
)

add_subdirectory(backward_vendor/backward-cpp)
add_subdirectory(gtest_vendor)
add_subdirectory(benchmark)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(plugins)
add_subdirectory(regression)
add_subdirectory(worlds)
