set(TEST_TYPE "BENCHMARK")
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  gz_add_benchmarks(SOURCES ${tests})
endif()
