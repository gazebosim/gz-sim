if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()

set(PYBIND11_PYTHON_VERSION 3)

find_package(pybind11 2.2 QUIET)

if (${pybind11_FOUND})
  message(STATUS "Building pybind11 interfaces")
  # Split from main extension and converted to pybind11
  pybind11_add_module(math SHARED
    src/_ignition_math_pybind11.cc
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

  # Set the build location and install location for a CPython extension
  function(configure_build_install_location _library_name)
    # Install into test folder in build space for unit tests to import
    set_target_properties(${_library_name} PROPERTIES
      # Use generator expression to avoid prepending a build type specific directory on Windows
      LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test>
      RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test>)

    # Install library for actual use
    install(TARGETS ${_library_name}
      DESTINATION "${IGN_PYTHON_INSTALL_PATH}/"
    )
  endfunction()

  configure_build_install_location(math)

  if (BUILD_TESTING)
    # Add the Python tests
    set(python_tests
      Vector2_TEST
      Vector3_TEST
      Vector4_TEST
    )

    foreach (test ${python_tests})
      add_test(NAME ${test}.py COMMAND
        "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/src/python_pybind11/test/${test}.py")

      set(_env_vars)
      list(APPEND _env_vars "PYTHONPATH=${FAKE_INSTALL_PREFIX}/lib/python/")
      list(APPEND _env_vars "LD_LIBRARY_PATH=${FAKE_INSTALL_PREFIX}/lib:$ENV{LD_LIBRARY_PATH}")
      set_tests_properties(${test}.py PROPERTIES
        ENVIRONMENT "${_env_vars}")
    endforeach()

  endif()
endif()
