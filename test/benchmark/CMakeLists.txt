set(TEST_TYPE "BENCHMARK")

find_package(benchmark)
include(IgnBenchmark OPTIONAL RESULT_VARIABLE IgnBenchmark_FOUND)

if (IgnBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  ign_add_benchmarks(SOURCES ${tests})

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()
