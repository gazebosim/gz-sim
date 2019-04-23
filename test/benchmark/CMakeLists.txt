set(TEST_TYPE "BENCHMARK")

find_package(benchmark)
include(IgnBenchmark OPTIONAL RESULT_VARIABLE IgnBenchmark_FOUND)

if (NOT benchmark_FOUND)
  message(WARNING "Unable to find google benchmark (libbenchmark-dev). Disabling benchmarks.")
endif()


if (NOT IgnBenchmark_FOUND)
  message(WARNING "Unable to find IgnBenchmark in ign-cmake. Disabling benchmarks.")
endif()

if (benchmark_FOUND AND IgnBenchmark_FOUND AND BUILD_TESTING)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  ign_build_executables(
    PREFIX "${TEST_TYPE}_"
    SOURCES ${tests}
    LIB_DEPS benchmark::benchmark
    EXEC_LIST benchmark_list
  )

  ign_add_run_benchmarks_target(TARGETS ${benchmark_list})
endif()
