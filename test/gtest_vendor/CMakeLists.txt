add_library(gtest STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/gtest-all.cc)
add_library(gtest_main STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/gtest_main.cc)

target_include_directories(gtest 
  SYSTEM PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(gtest_main gtest)
