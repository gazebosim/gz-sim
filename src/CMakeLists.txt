include (${project_cmake_dir}/Utils.cmake)

set (sources
  Angle.cc
  Box.cc
  Helpers.cc
  Matrix3.cc
  Matrix4.cc
  Plane.cc
  Pose.cc
  Quaternion.cc
  Rand.cc
  RotationSpline.cc
  Spline.cc
  Vector2d.cc
  Vector2i.cc
  Vector3.cc
  Vector4.cc
)

set (gtest_sources
  Angle_TEST.cc
  Box_TEST.cc
  Helpers_TEST.cc
  Matrix3_TEST.cc
  Matrix4_TEST.cc
  Plane_TEST.cc
  Pose_TEST.cc
  Quaternion_TEST.cc
  Rand_TEST.cc
  RotationSpline_TEST.cc
  Spline_TEST.cc
  Vector2d_TEST.cc
  Vector2i_TEST.cc
  Vector3_TEST.cc
  Vector4_TEST.cc
)

ign_build_tests(${gtest_sources})

ign_add_library(ignition_math ${sources})
target_link_libraries(ignition_math ${Boost_LIBRARIES})

ign_install_library(ignition_math)
