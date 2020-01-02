cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition-Math library
set(IGN_MATH_VER 6)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

add_executable(angle_example angle_example.cc)
target_link_libraries(angle_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(vector2_example vector2_example.cc)
target_link_libraries(vector2_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(triangle_example triangle_example.cc)
target_link_libraries(triangle_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(quaternion_to_euler quaternion_to_euler.cc)
target_link_libraries(quaternion_to_euler ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(quaternion_from_euler quaternion_from_euler.cc)
target_link_libraries(quaternion_from_euler ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(graph_example graph_example.cc)
target_link_libraries(graph_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})
