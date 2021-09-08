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

  set(swig_files
    Angle
    GaussMarkovProcess
    PID
    Rand
    SemanticVersion
    Vector2
    Vector3
    Vector4)
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
  install(TARGETS ${SWIG_PY_LIB} DESTINATION ${IGN_LIB_INSTALL_DIR}/python/ignition)
  install(FILES ${CMAKE_BINARY_DIR}/lib/python/math.py DESTINATION ${IGN_LIB_INSTALL_DIR}/python/ignition)

  if (BUILD_TESTING)
    # Add the Python tests
    set(python_tests
      Angle_TEST
      GaussMarkovProcess_TEST
      Kmeans_TEST
      Line2_TEST
      Line3_TEST
      PID_TEST
      python_TEST
      Rand_TEST
      SemanticVersion_TEST
      SignalStats_TEST
      Vector2_TEST
      Vector3_TEST
      Vector3Stats_TEST
      Vector4_TEST
      Temperature_TEST
      Triangle_TEST
    )

    foreach (test ${python_tests})
      add_test(NAME ${test}.py COMMAND
        python3 ${CMAKE_SOURCE_DIR}/src/python/${test}.py)

      set(_env_vars)
      list(APPEND _env_vars "PYTHONPATH=${FAKE_INSTALL_PREFIX}/lib/python/")
      list(APPEND _env_vars "LD_LIBRARY_PATH=${FAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
      set_tests_properties(${test}.py PROPERTIES
        ENVIRONMENT "${_env_vars}")
    endforeach()
  endif()

endif()
