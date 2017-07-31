include (${project_cmake_dir}/Utils.cmake)

set (sources
  Example.cc
)

set (gtest_sources
  Example_TEST.cc
)

ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})

set_target_properties(${PROJECT_LIBRARY_TARGET_NAME} PROPERTIES
  CXX_STANDARD 14 
  CXX_STANDARD_REQURED YES)

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>)

ign_build_tests(${gtest_sources})
ign_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})
