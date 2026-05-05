set(TEST_TYPE "BENCHMARK")
find_package(benchmark)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  # Archetype-ECS microbenches exercise gz::sim::ecs::World templates
  # directly (C++20). They are only meaningful when the archetype
  # backend is selected, so they are added to the bench list only
  # under GZ_SIM_ARCHETYPE_ECM=ON.
  if (GZ_SIM_ARCHETYPE_ECM)
    list(APPEND tests
      bench_iteration.cc
      bench_parallel.cc
      bench_transitions.cc
      bench_vs_ecm.cc
    )
  endif()

  gz_add_benchmarks(SOURCES ${tests})

  # The archetype benches use C++20 templates from gz-sim-ecs.
  # gz_add_benchmarks links every bench against gz-sim by default
  # (which exposes the archetype core via --whole-archive under the
  # archetype build), so no extra link is needed — but the consumer
  # TUs still need the C++20 standard set.
  if (GZ_SIM_ARCHETYPE_ECM)
    foreach(bench_name bench_iteration bench_parallel bench_transitions
                       bench_vs_ecm)
      set(bench_target BENCHMARK_${bench_name})
      if (TARGET ${bench_target})
        target_compile_features(${bench_target} PRIVATE cxx_std_20)
      endif()
    endforeach()
  endif()

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()
