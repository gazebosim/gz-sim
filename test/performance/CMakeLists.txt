set(TEST_TYPE "PERFORMANCE")

find_package(benchmark REQUIRED)

set(tests
  each.cc
  level_manager.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)


add_executable(PERFORMANCE_each
  each.cc
)

target_link_libraries(PERFORMANCE_each
  benchmark
  benchmark_main
  ignition-gazebo${PROJECT_VERSION_MAJOR}
)
