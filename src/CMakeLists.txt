
# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
ign_get_libsources_and_unittests(sources gtest_sources)

# Create the library target
ign_create_core_library(SOURCES ${sources} CXX_STANDARD ${c++standard})

# Build the unit tests
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources})

# graph namespace
add_subdirectory(graph)

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
    Rand
    Vector2
    Vector3
    Vector4)
endif()

#################################################
# Create and install Ruby interfaces
# Example usage
# $ export RUBYLIB=/usr/local/lib/ruby
# $ ruby -e "require 'ignition/math'; a = Ignition::Math::Angle.new(20); puts a.Degree()"
if (RUBY_FOUND)
  foreach (swig_file ${swig_files})
    # Assuming that each swig file has a test
    list(APPEND ruby_tests ${swig_file}_TEST)

    # Generate the list if .i files
    list(APPEND swig_i_files ${swig_file}.i)
  endforeach()
  list(APPEND ruby_tests ruby_TEST)

  # Turn on c++
  set_source_files_properties(${swig_i_files} ruby/ruby.i PROPERTIES CPLUSPLUS ON)

  # Create the ruby library

  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/ruby")
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    SWIG_ADD_LIBRARY(math LANGUAGE ruby SOURCES ruby/ruby.i ${swig_i_files})
  else()
    SWIG_ADD_MODULE(math ruby ruby/ruby.i ${swig_i_files})
  endif()

  # Suppress warnings on SWIG-generated files
  target_compile_options(math PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-pedantic -Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter>
    $<$<CXX_COMPILER_ID:AppleClang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter>
  )
  target_include_directories(math SYSTEM PUBLIC ${RUBY_INCLUDE_DIRS})

  SWIG_LINK_LIBRARIES(math
    ${RUBY_LIBRARY}
    ignition-math${PROJECT_VERSION_MAJOR}
  )
  target_compile_features(math PUBLIC ${IGN_CXX_${c++standard}_FEATURES})
  install(TARGETS math DESTINATION ${IGN_LIB_INSTALL_DIR}/ruby/ignition)

  # Add the ruby tests
  foreach (test ${ruby_tests})
    add_test(NAME ${test}.rb COMMAND
      ruby -I${CMAKE_BINARY_DIR}/lib ${CMAKE_SOURCE_DIR}/src/${test}.rb
   	  --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${test}rb.xml)
  endforeach()
endif()

#################################
# Create and install Python interfaces
# Example usage
# $ export PYTHONPATH=/ws/install/lib/python/:$PYTHONPATH
if (PYTHONLIBS_FOUND)
  set_source_files_properties(python/python.i PROPERTIES CPLUSPLUS ON)
  set_source_files_properties(python/python.i PROPERTIES SWIG_FLAGS "-includeall")
  set_source_files_properties(python/python.i PROPERTIES SWIG_MODULE_NAME "math")
  set(SWIG_PY_LIB pymath)
  set(SWIG_PY_LIB_OUTPUT math)

  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    SWIG_ADD_LIBRARY(${SWIG_PY_LIB} LANGUAGE python SOURCES python/python.i)
  else()
    SWIG_ADD_MODULE(${SWIG_PY_LIB} python python/python.i)
  endif()

  SWIG_LINK_LIBRARIES(${SWIG_PY_LIB}
    ${PYTHON_LIBRARIES}
    ignition-math${PROJECT_VERSION_MAJOR}
  )

  # When using SWIG 3 to build a python interface there is an extra underscore between the name of
  # the library given to swig_add_library macro and the generated .so
  if(NOT ${SWIG_VERSION} VERSION_GREATER 4.0.0)
    set(SWIG_PY_LIB "_${SWIG_PY_LIB}")
    set(SWIG_PY_LIB_OUTPUT "_${SWIG_PY_LIB_OUTPUT}")
  endif()

  set_target_properties(${SWIG_PY_LIB}
    PROPERTIES
      OUTPUT_NAME ${SWIG_PY_LIB_OUTPUT}
  )

  # Suppress warnings on SWIG-generated files
  target_compile_options(${SWIG_PY_LIB} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-pedantic -Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-cast-function-type -Wno-missing-field-initializers>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-cast-function-type -Wno-missing-field-initializers>
    $<$<CXX_COMPILER_ID:AppleClang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter -Wno-cast-function-type -Wno-missing-field-initializers>
  )
  install(TARGETS ${SWIG_PY_LIB} DESTINATION ${IGN_LIB_INSTALL_DIR}/python/ignition)
  install(FILES ${CMAKE_BINARY_DIR}/lib/python/math.py DESTINATION ${IGN_LIB_INSTALL_DIR}/python/ignition)

  if (BUILD_TESTING)
    # Add the Python tests
    set(python_tests
      Angle_TEST
      GaussMarkovProcess_TEST
      python_TEST
      Rand_TEST
      Vector2_TEST
      Vector3_TEST
      Vector4_TEST
    )

    foreach (test ${python_tests})
      add_test(NAME ${test}.py COMMAND
        python3 ${CMAKE_SOURCE_DIR}/src/${test}.py)

      set(_env_vars)
      list(APPEND _env_vars "PYTHONPATH=${FAKE_INSTALL_PREFIX}/lib/python/")
      list(APPEND _env_vars "LD_LIBRARY_PATH=${FAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
      set_tests_properties(${test}.py PROPERTIES
        ENVIRONMENT "${_env_vars}")
    endforeach()
  endif()

endif()
