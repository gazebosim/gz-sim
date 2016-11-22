include (${project_cmake_dir}/Utils.cmake)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set (sources
  Angle.cc
  Box.cc
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if (SWIG_FOUND)
  
  set(swig_files
    Angle.i
    Vector2.i
    Vector3.i)
endif()

#################################################
# Create and install Ruby interfaces
# Example usage
# $ export RUBYLIB=/usr/local/lib/ruby
# $ ruby -e "require 'ignition/math'; a = Ignition::Math::Angle.new(20); puts a.Degree()"
if (RUBY_FOUND)
  include_directories(${RUBY_INCLUDE_DIRS})

  set(ruby_tests
    Angle_TEST
    Vector3_TEST
  )

  set_source_files_properties(ruby.i ${swig_files} PROPERTIES CPLUSPLUS ON)
  
  SWIG_ADD_MODULE(math ruby ruby.i ${swig_files} ${sources})
  SWIG_LINK_LIBRARIES(math ${RUBY_LIBRARY})
  install(TARGETS math DESTINATION ${LIB_INSTALL_DIR}/ruby/ignition)
  
  foreach (test ${ruby_tests})
    add_test(NAME ${test} COMMAND
      ruby -I${CMAKE_BINARY_DIR}/src ${CMAKE_SOURCE_DIR}/src/${test}.rb
  	  --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${test}rb.xml)
  endforeach()
endif()
