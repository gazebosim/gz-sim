#################################################
# Setup swig
if (SWIG_FOUND)
  if (POLICY CMP0078)
    cmake_policy(SET CMP0078 NEW)
  endif()
  if (POLICY CMP0086)
    cmake_policy(SET CMP0086 NEW)
  endif()

  include(${SWIG_USE_FILE})
  set(CMAKE_SWIG_FLAGS "")

  include_directories(${PROJECT_SOURCE_DIR}/include)
  include_directories(${PYTHON_INCLUDE_PATH})
endif()

#################################
# Create and install Python interfaces
# Example usage
# $ export PYTHONPATH=/ws/install/lib/python/:$PYTHONPATH
if (PYTHONLIBS_FOUND)
  set_source_files_properties(python.i PROPERTIES CPLUSPLUS ON)
  set_source_files_properties(python.i PROPERTIES SWIG_FLAGS "-includeall")
  set_source_files_properties(python.i PROPERTIES SWIG_MODULE_NAME "math")
  set(SWIG_PY_LIB pymath)
  set(SWIG_PY_LIB_OUTPUT math)

  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    SWIG_ADD_LIBRARY(${SWIG_PY_LIB} LANGUAGE python SOURCES python.i)
  else()
    SWIG_ADD_MODULE(${SWIG_PY_LIB} python python.i)
  endif()

  SWIG_LINK_LIBRARIES(${SWIG_PY_LIB}
    ${PYTHON_LIBRARIES}
    ignition-math${PROJECT_VERSION_MAJOR}
  )

  if(NOT CMAKE_VERSION VERSION_GREATER_EQUAL 3.13.0)
    set(SWIG_PY_LIB "_${SWIG_PY_LIB}")
    set(SWIG_PY_LIB_OUTPUT "_${SWIG_PY_LIB_OUTPUT}")
  endif()

  set_target_properties(${SWIG_PY_LIB}
    PROPERTIES
      OUTPUT_NAME ${SWIG_PY_LIB_OUTPUT}
  )

  # Suppress warnings on SWIG-generated files
  target_compile_options(${SWIG_PY_LIB} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-pedantic -Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-cast-function-type -Wno-missing-field-initializers -Wno-class-memaccess>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-cast-function-type -Wno-missing-field-initializers -Wno-class-memaccess>
    $<$<CXX_COMPILER_ID:AppleClang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-cast-function-type -Wno-missing-field-initializers -Wno-class-memaccess>
  )

  if(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION)
    if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
      execute_process(
        COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
	  from distutils import sysconfig as sc
	  print(sc.get_python_lib(plat_specific=True))"
        OUTPUT_VARIABLE Python3_SITEARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
      # Get install variable from Python3 module
      # Python3_SITEARCH is available from 3.12 on, workaround if needed:
      find_package(Python3 COMPONENTS Interpreter)
    endif()

    if(USE_DIST_PACKAGES_FOR_PYTHON)
      string(REPLACE "site-packages" "dist-packages" IGN_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
    else()
      # custom cmake command is returning dist-packages
      string(REPLACE "dist-packages" "site-packages" IGN_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
    endif()
  else()
    # If not a system installation, respect local paths
    set(IGN_PYTHON_INSTALL_PATH ${IGN_LIB_INSTALL_DIR}/python)
  endif()

  set(IGN_PYTHON_INSTALL_PATH "${IGN_PYTHON_INSTALL_PATH}/ignition")
  install(TARGETS ${SWIG_PY_LIB} DESTINATION ${IGN_PYTHON_INSTALL_PATH})
  install(FILES ${CMAKE_BINARY_DIR}/lib/python/math.py DESTINATION ${IGN_PYTHON_INSTALL_PATH})

  if (BUILD_TESTING)
    # Add the Python tests
    set(python_tests
      Angle_TEST
      AxisAlignedBox_TEST
      Box_TEST
      Color_TEST
      Cylinder_TEST
      DiffDriveOdometry_TEST
      Filter_TEST
      GaussMarkovProcess_TEST
      Helpers_TEST
      Kmeans_TEST
      Line2_TEST
      Line3_TEST
      MassMatrix3_TEST
      Material_TEST
      Matrix3_TEST
      Matrix4_TEST
      MovingWindowFilter_TEST
      PID_TEST
      Plane_TEST
      Pose3_TEST
      python_TEST
      Quaternion_TEST
      Rand_TEST
      RollingMean_TEST
      RotationSpline_TEST
      SemanticVersion_TEST
      SignalStats_TEST
      Sphere_TEST
      SphericalCoordinates_TEST
      Spline_TEST
      StopWatch_TEST
      Temperature_TEST
      Triangle_TEST
      Triangle3_TEST
      Vector2_TEST
      Vector3_TEST
      Vector3Stats_TEST
      Vector4_TEST
    )

    foreach (test ${python_tests})
      add_test(NAME ${test}.py COMMAND
        "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/src/python/${test}.py")

      set(_env_vars)
      list(APPEND _env_vars "PYTHONPATH=${FAKE_INSTALL_PREFIX}/lib/python/")
      list(APPEND _env_vars "LD_LIBRARY_PATH=${FAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
      set_tests_properties(${test}.py PROPERTIES
        ENVIRONMENT "${_env_vars}")
    endforeach()
  endif()

endif()
