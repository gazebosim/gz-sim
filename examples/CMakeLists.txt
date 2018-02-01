cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition-Math library
find_package(ignition-math5 REQUIRED)

add_executable(vector2_example vector2_example.cc)
target_link_libraries(vector2_example ignition-math4::ignition-math4)

add_executable(triangle_example triangle_example.cc)
target_link_libraries(triangle_example ignition-math4::ignition-math4)

add_executable(graph_example graph_example.cc)
target_link_libraries(graph_example ignition-math4::ignition-math4)
