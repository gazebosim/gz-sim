set(TEST_TYPE "BENCHMARK")

# TODO(CH3): Deprecated. Remove on ticktock
include(IgnBenchmark OPTIONAL RESULT_VARIABLE IgnBenchmark_FOUND)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND OR IgnBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  ign_add_benchmarks(SOURCES ${tests})
endif()
