set(TEST_TYPE "BENCHMARK")
find_package(benchmark)
include(GzBenchmark OPTIONAL RESULT_VARIABLE GzBenchmark_FOUND)

if (GzBenchmark_FOUND)
  set(tests
    each.cc
    ecm_serialize.cc
  )

  gz_add_benchmarks(SOURCES ${tests})

  if (benchmark_VERSION VERSION_LESS "1.9.5")
    add_compile_definitions(BENCHMARK_INTERNAL)
  endif()

endif()

# Archetype-ECS microbenches. These exercise the archetype core
# (gz::sim::ecs::World) directly and only make sense when that
# backend is enabled. They use a dependency-free in-process timing
# helper (bench_common.hh) rather than google-benchmark, so they
# are built as plain executables independent of GzBenchmark.
if (GZ_SIM_ARCHETYPE_ECM)
  set(ECS_BENCH_SOURCES
    bench_iteration.cc
    bench_parallel.cc
    bench_transitions.cc
  )
  foreach(bench_src ${ECS_BENCH_SOURCES})
    get_filename_component(bench_name ${bench_src} NAME_WE)
    add_executable(${bench_name} ${bench_src})
    target_link_libraries(${bench_name} PRIVATE gz-sim-ecs pthread)
    target_include_directories(${bench_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(${bench_name} PRIVATE -O3 -DNDEBUG)
  endforeach()

  # Head-to-head bench against the public EntityComponentManager
  # (which under GZ_SIM_ARCHETYPE_ECM=ON is itself archetype-backed,
  # so this measures the facade overhead vs direct ecs::World use).
  add_executable(bench_vs_ecm bench_vs_ecm.cc)
  target_link_libraries(bench_vs_ecm PRIVATE
    gz-sim-ecs
    ${PROJECT_LIBRARY_TARGET_NAME}
    pthread)
  target_include_directories(bench_vs_ecm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_options(bench_vs_ecm PRIVATE -O3 -DNDEBUG)
endif()
