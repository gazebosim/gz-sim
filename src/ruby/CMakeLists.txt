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

  # Create the ruby library

  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/ruby")
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    SWIG_ADD_LIBRARY(math LANGUAGE ruby SOURCES ruby.i ${swig_i_files})
  else()
    SWIG_ADD_MODULE(math ruby ruby.i ${swig_i_files})
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
      ruby -I${CMAKE_BINARY_DIR}/lib ${CMAKE_SOURCE_DIR}/src/ruby/${test}.rb
   	  --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${test}rb.xml)
  endforeach()
endif()
