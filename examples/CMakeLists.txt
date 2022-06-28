cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(gz-math-examples)

# Find the Gazebo Math library
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

add_executable(additively_separable_scalar_field3_example additively_separable_scalar_field3_example.cc)
target_link_libraries(additively_separable_scalar_field3_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(angle_example angle_example.cc)
target_link_libraries(angle_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(color_example color_example.cc)
target_link_libraries(color_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(diff_drive_odometry diff_drive_odometry.cc)
target_link_libraries(diff_drive_odometry gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(gauss_markov_process gauss_markov_process_example.cc)
target_link_libraries(gauss_markov_process gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(graph_example graph_example.cc)
target_link_libraries(graph_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(helpers_example helpers_example.cc)
target_link_libraries(helpers_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(interval_example interval_example.cc)
target_link_libraries(interval_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(kmeans kmeans.cc)
target_link_libraries(kmeans gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(matrix3_example matrix3_example.cc)
target_link_libraries(matrix3_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(piecewise_scalar_field3_example piecewise_scalar_field3_example.cc)
target_link_libraries(piecewise_scalar_field3_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(polynomial3_example polynomial3_example.cc)
target_link_libraries(polynomial3_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(pose3_example pose3_example.cc)
target_link_libraries(pose3_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(quaternion_example quaternion_example.cc)
target_link_libraries(quaternion_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(quaternion_from_euler quaternion_from_euler.cc)
target_link_libraries(quaternion_from_euler gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(quaternion_to_euler quaternion_to_euler.cc)
target_link_libraries(quaternion_to_euler gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(rand_example rand_example.cc)
target_link_libraries(rand_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(region3_example region3_example.cc)
target_link_libraries(region3_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(temperature_example temperature_example.cc)
target_link_libraries(temperature_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(triangle_example triangle_example.cc)
target_link_libraries(triangle_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})

add_executable(vector2_example vector2_example.cc)
target_link_libraries(vector2_example gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER})
