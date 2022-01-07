cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ignition-math-examples)

# Find the Ignition-Math library
set(IGN_MATH_VER 6)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

add_executable(angle_example angle_example.cc)
target_link_libraries(angle_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(diff_drive_odometry diff_drive_odometry.cc)
target_link_libraries(diff_drive_odometry ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(gauss_markov_process gauss_markov_process_example.cc)
target_link_libraries(gauss_markov_process ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(vector2_example vector2_example.cc)
target_link_libraries(vector2_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(triangle_example triangle_example.cc)
target_link_libraries(triangle_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(quaternion_to_euler quaternion_to_euler.cc)
target_link_libraries(quaternion_to_euler ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(quaternion_from_euler quaternion_from_euler.cc)
target_link_libraries(quaternion_from_euler ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(rand_example rand_example.cc)
target_link_libraries(rand_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(graph_example graph_example.cc)
target_link_libraries(graph_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(kmeans kmeans.cc)
target_link_libraries(kmeans ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

add_executable(color_example color_example.cc)
target_link_libraries(color_example ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})
