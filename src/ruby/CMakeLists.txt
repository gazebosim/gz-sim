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
  set_source_files_properties(${swig_i_files} ruby.i PROPERTIES CPLUSPLUS ON)
  set(SWIG_RB_LIB rbmath)
  set(SWIG_RB_LIB_OUTPUT math)

  # Create the ruby library
  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/ruby")
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    SWIG_ADD_LIBRARY(${SWIG_RB_LIB} LANGUAGE ruby SOURCES ruby.i ${swig_i_files})
  else()
    SWIG_ADD_MODULE(${SWIG_RB_LIB} ruby ruby.i ${swig_i_files})
  endif()

  # Suppress warnings on SWIG-generated files
  target_compile_options(${SWIG_RB_LIB} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wno-pedantic -Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter>
    $<$<CXX_COMPILER_ID:Clang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter>
    $<$<CXX_COMPILER_ID:AppleClang>:-Wno-shadow -Wno-maybe-uninitialized -Wno-unused-parameter>
  )
  target_include_directories(${SWIG_RB_LIB} SYSTEM PUBLIC ${RUBY_INCLUDE_DIRS})

  SWIG_LINK_LIBRARIES(${SWIG_RB_LIB}
    ${RUBY_LIBRARY}
    ignition-math${PROJECT_VERSION_MAJOR}
  )
  target_compile_features(${SWIG_RB_LIB} PUBLIC ${IGN_CXX_${c++standard}_FEATURES})

  set_target_properties(${SWIG_RB_LIB}
    PROPERTIES
      OUTPUT_NAME ${SWIG_RB_LIB_OUTPUT}
  )

  if(USE_SYSTEM_PATHS_FOR_RUBY_INSTALLATION)
    execute_process(
      COMMAND ${RUBY_EXECUTABLE} -r rbconfig -e "print RbConfig::CONFIG['vendordir']"
      OUTPUT_VARIABLE IGN_RUBY_INSTALL_PATH
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  else()
      set(IGN_RUBY_INSTALL_PATH ${IGN_LIB_INSTALL_DIR}/ruby)
  endif()
  set(IGN_RUBY_INSTALL_PATH "${IGN_RUBY_INSTALL_PATH}/ignition")
  install(TARGETS ${SWIG_RB_LIB} DESTINATION ${IGN_RUBY_INSTALL_PATH})

  # Add the ruby tests
  set(_env_vars)
  list(APPEND _env_vars "LD_LIBRARY_PATH=${FAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
  foreach (test ${ruby_tests})
    add_test(NAME ${test}.rb COMMAND
      ruby -I${FAKE_INSTALL_PREFIX}/lib/ruby/ignition ${CMAKE_SOURCE_DIR}/src/ruby/${test}.rb
   	  --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${test}rb.xml)
    set_tests_properties(${test}.rb PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endforeach()
endif()
