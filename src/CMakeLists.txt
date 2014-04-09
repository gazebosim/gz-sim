include (${project_cmake_dir}/Utils.cmake)

set (sources
  Angle.cc
  Box.cc
  Helpers.cc
  Matrix3d.cc
  Matrix3f.cc
  Matrix3i.cc
  Matrix4d.cc
  Matrix4f.cc
  Matrix4i.cc
  Plane.cc
  Pose3d.cc
  Pose3f.cc
  Pose3i.cc
  Quaterniond.cc
  Quaternionf.cc
  Quaternioni.cc
  Rand.cc
  RotationSpline.cc
  Spline.cc
  Vector2d.cc
  Vector2f.cc
  Vector2i.cc
  Vector3d.cc
  Vector3f.cc
  Vector3i.cc
  Vector4d.cc
  Vector4f.cc
  Vector4i.cc
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
  Vector2_TEST.cc
  Vector3_TEST.cc
  Vector4_TEST.cc
)

ign_build_tests(${gtest_sources})
ign_add_library(ignition_math ${sources})
ign_install_library(ignition_math)
