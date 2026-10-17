set(TEST_TYPE "BENCHMARK")
find_package(benchmark)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
    planar_collision.cc
    server_run.cc
    utils_benchmark.cc
  )

  gz_add_benchmarks(SOURCES ${tests})

  # The planar collision queries are internal to the simplified physics system.
  if (TARGET BENCHMARK_planar_collision)
    target_link_libraries(BENCHMARK_planar_collision
      gz-sim-planar-collision)
    target_include_directories(BENCHMARK_planar_collision
      PRIVATE ${PROJECT_SOURCE_DIR}/src)
  endif()

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()
