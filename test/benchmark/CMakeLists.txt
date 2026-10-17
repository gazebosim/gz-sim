set(TEST_TYPE "BENCHMARK")
find_package(benchmark)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    diff_drive.cc
    each.cc
    ecm_serialize.cc
    server_run.cc
    utils_benchmark.cc
  )

  gz_add_benchmarks(SOURCES ${tests})

  # The DiffDrive benchmark calls the system directly, without a server.
  if (TARGET BENCHMARK_diff_drive)
    target_sources(BENCHMARK_diff_drive PRIVATE
      ${PROJECT_SOURCE_DIR}/src/systems/diff_drive/DiffDrive.cc)
    target_include_directories(BENCHMARK_diff_drive PRIVATE
      ${PROJECT_SOURCE_DIR}/src/systems/diff_drive)
  endif()

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()
