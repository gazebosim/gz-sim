include (${project_cmake_dir}/Utils.cmake)

set (sources
)

set (gtest_sources
)

#################################################
### Do not change the following

ign_build_tests(${gtest_sources})
ign_add_library(${PROJECT_NAME_LOWER} ${sources})
target_link_libraries(${PROJECT_NAME_LOWER} ${Boost_LIBRARIES})
ign_install_library(${PROJECT_NAME_LOWER})
