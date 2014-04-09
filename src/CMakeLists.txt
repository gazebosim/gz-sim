include (${project_cmake_dir}/Utils.cmake)

set (sources
)

set (gtest_sources
)

ign_build_tests(${gtest_sources})
ign_add_library(${PROJECT_NAME_LOWER} ${sources})
ign_install_library(${PROJECT_NAME_LOWER})
