set(TEST_TYPE "BENCHMARK")
find_package(benchmark)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  # Archetype-ECS microbenches. These build under either backend so
  # the BM_Legacy* / BM_Archetype* head-to-head benches inside each
  # file produce a meaningful comparison:
  #   * Under GZ_SIM_ARCHETYPE_ECM=OFF, BM_Legacy* exercises the
  #     legacy unordered_map ECM and BM_Archetype* exercises the
  #     standalone gz-sim-ecs core — true legacy-vs-archetype.
  #   * Under GZ_SIM_ARCHETYPE_ECM=ON, BM_Legacy* exercises the
  #     archetype-backed ECM facade and BM_Archetype* exercises the
  #     core directly — facade-vs-direct.
  list(APPEND tests
    archetype_iteration.cc
    archetype_parallel.cc
    archetype_transitions.cc
    archetype_vs_ecm.cc
  )

  gz_add_benchmarks(SOURCES ${tests})

  # The archetype benches use C++20 templates from gz-sim-ecs and
  # need the standalone library linked explicitly: under
  # GZ_SIM_ARCHETYPE_ECM=OFF gz-sim doesn't pull in gz-sim-ecs, so
  # the bench TUs would otherwise see undefined references. Under
  # =ON the static lib is already absorbed into libgz-sim via
  # --whole-archive, so the explicit link is harmless (the linker
  # deduplicates).
  foreach(bench_name archetype_iteration archetype_parallel
                     archetype_transitions archetype_vs_ecm)
    set(bench_target BENCHMARK_${bench_name})
    if (TARGET ${bench_target})
      target_compile_features(${bench_target} PRIVATE cxx_std_20)
      # Use the plain target_link_libraries signature to match
      # the call inside gz_build_executables (cmake forbids
      # mixing plain and keyword signatures on the same target).
      target_link_libraries(${bench_target} gz-sim-ecs)
    endif()
  endforeach()

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()
