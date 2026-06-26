set(TEST_TYPE "BENCHMARK")
find_package(benchmark)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
<<<<<<< HEAD
=======
    server_run.cc
    utils_benchmark.cc
>>>>>>> 13adaf7c (Add benchmark for Server.Run operation  (#3671))
  )

  gz_add_benchmarks(SOURCES ${tests})

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()
