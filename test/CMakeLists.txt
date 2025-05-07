include_directories (
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
  ${PROJECT_BINARY_DIR}/include/
)

<<<<<<< HEAD
configure_file (test_config.hh.in ${PROJECT_BINARY_DIR}/include/ignition/gazebo/test_config.hh)
configure_file (test_config.hh.in ${PROJECT_BINARY_DIR}/include/gz/sim/test_config.hh)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
target_compile_features(gtest PUBLIC cxx_std_11)

set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

=======
add_subdirectory(backward_vendor/backward-cpp)
add_subdirectory(gtest_vendor)
>>>>>>> 7608ed9d (Attempts to print a stacktrace on segfault in our integration tests (#2776))
add_subdirectory(benchmark)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(plugins)
add_subdirectory(regression)
add_subdirectory(worlds)
