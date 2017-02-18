include (${project_cmake_dir}/Utils.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set (sources
  Angle.cc
  Box.cc
  Color.cc
  Frustum.cc
  Helpers.cc
  IndexException.cc
  Kmeans.cc
  PID.cc
  Rand.cc
  RotationSpline.cc
  RotationSplinePrivate.cc
  SemanticVersion.cc
  SignalStats.cc
  SphericalCoordinates.cc
  Spline.cc
  Temperature.cc
  Vector3Stats.cc
)

set (gtest_sources
  Angle_TEST.cc
  Box_TEST.cc
  Color_TEST.cc
  Filter_TEST.cc
  Frustum_TEST.cc
  Helpers_TEST.cc
  Inertial_TEST.cc
  Kmeans_TEST.cc
  Line2_TEST.cc
  Line3_TEST.cc
  MassMatrix3_TEST.cc
  Matrix3_TEST.cc
  Matrix4_TEST.cc
  OrientedBox_TEST.cc
  PID_TEST.cc
  Plane_TEST.cc
  Pose_TEST.cc
  Quaternion_TEST.cc
  Rand_TEST.cc
  RotationSpline_TEST.cc
  SemanticVersion_TEST.cc
  SignalStats_TEST.cc
  SphericalCoordinates_TEST.cc
  Spline_TEST.cc
  Temperature_TEST.cc
  Triangle_TEST.cc
  Triangle3_TEST.cc
  Vector2_TEST.cc
  Vector3_TEST.cc
  Vector3Stats_TEST.cc
  Vector4_TEST.cc
)

ign_add_library(ignition-math${PROJECT_MAJOR_VERSION} ${sources})
ign_build_tests(${gtest_sources})
ign_install_library(ignition-math${PROJECT_MAJOR_VERSION})
