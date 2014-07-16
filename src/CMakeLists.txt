include (${project_cmake_dir}/Utils.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set (sources
  Angle.cc
  Box.cc
  Helpers.cc
  Rand.cc
  RotationSpline.cc
  Spline.cc
)

set (gtest_sources
  Angle_TEST.cc
  Box_TEST.cc
  Filter_TEST.cc
  Helpers_TEST.cc
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

ign_add_library(ignition_math ${sources})
ign_build_tests(${gtest_sources})
ign_install_library(ignition_math)

set_source_files_properties(ign_math.i Angle.i Vector2.i Vector3.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(ign_math.i Angle.i Vector2.i Vector3.i PROPERTIES SWIG_FLAGS "-includeall")
SWIG_ADD_MODULE(ign_math python ign_math.i Angle.i Vector2.i Vector3.i Angle.cc Box.cc)
SWIG_LINK_LIBRARIES(ign_math ${PYTHON_LIBRARIES})
