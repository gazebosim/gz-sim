set(TEST_TYPE "BENCHMARK")

set(tests
  each.cc
  ecm_serialize.cc
)

ign_build_executables(
  PREFIX "${TEST_TYPE}_"
  SOURCES ${tests}
  LIB_DEPS gbenchmark gbenchmark_main
  EXEC_LIST test_list
)
