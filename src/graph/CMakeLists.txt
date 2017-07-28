include (${project_cmake_dir}/Utils.cmake)

set (sources
)

set (gtest_sources
  Edge_TEST.cc
  Graph_TEST.cc
  GraphDirected_TEST.cc
  GraphUndirected_TEST.cc
  Vertex_TEST.cc
)

ign_build_tests(${gtest_sources})
