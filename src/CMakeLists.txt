include (${project_cmake_dir}/Utils.cmake)

set (sources
  Angle.cc
  Box.cc
  Color.cc
  Frustum.cc
  Helpers.cc
  Kmeans.cc
  PID.cc
  Rand.cc
  RotationSpline.cc
  RotationSplinePrivate.cc
  SemanticVersion.cc
  SignalStats.cc
  SphericalCoordinates.cc
  Spline.cc
  SplinePrivate.cc
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

ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})

# The following line enables C++14 (or greater) mode for this library an all
# downstream libraries. "cxx_attribute_deprecated" is a feature that is only
# available in c++14 and after. This allows downstream libraries to use
# c++14 or c++17 and beyond.
#
# When switching to cmake 3.8, we can use that cxx_std_14 meta feature.
target_compile_features(${PROJECT_LIBRARY_TARGET_NAME}
                        PUBLIC cxx_attribute_deprecated)

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>)

ign_build_tests(${gtest_sources})
ign_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})

# graph namespace
add_subdirectory(graph)
