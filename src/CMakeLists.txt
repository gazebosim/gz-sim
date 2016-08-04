include (${project_cmake_dir}/Utils.cmake)

set (sources
  Angle.cc
  Box.cc
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

# When the minimum CMake required version will be >= 3.1 
# we could use the target_compile_features() command 
# to enable C++11 support in a platform-independent way. 
if(NOT MSVC)
    target_compile_options(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC "-std=c++11")
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR_FULL}>)

ign_build_tests(${gtest_sources})
ign_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})
