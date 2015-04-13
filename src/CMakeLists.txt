include (${project_cmake_dir}/Utils.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set (sources
  Angle.cc
  Box.cc
  BoxPrivate.cc
  Helpers.cc
  Kmeans.cc
  Rand.cc
  RotationSpline.cc
  RotationSplinePrivate.cc
  Spline.cc
)

set (gtest_sources
  Angle_TEST.cc
  Box_TEST.cc
  Filter_TEST.cc
  Helpers_TEST.cc
  Kmeans_TEST.cc
  Line2_TEST.cc
  Matrix3_TEST.cc
  Matrix4_TEST.cc
  Plane_TEST.cc
  Pose_TEST.cc
  Quaternion_TEST.cc
  Rand_TEST.cc
  RotationSpline_TEST.cc
  Spline_TEST.cc
  Triangle_TEST.cc
  Vector2_TEST.cc
  Vector3_TEST.cc
  Vector4_TEST.cc
)

ign_add_library(ignition-math ${sources})
ign_build_tests(${gtest_sources})
ign_install_library(ignition-math)
