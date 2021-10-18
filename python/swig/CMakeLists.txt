########################################
# Include swig
find_package(SWIG QUIET)
if (NOT SWIG_FOUND)
  IGN_BUILD_WARNING("Swig is missing: Language interfaces are disabled.")
  message (STATUS "Searching for swig - not found.")
else()
  message (STATUS "Searching for swig - found version ${SWIG_VERSION}.")
endif()

# Include other languages if swig was found
if (SWIG_FOUND)
  ########################################
  # Include ruby
  find_package(Ruby 1.9 QUIET)
  if (NOT RUBY_FOUND)
    IGN_BUILD_WARNING("Ruby is missing: Install ruby-dev to enable ruby interface to ignition math.")
    message (STATUS "Searching for Ruby - not found.")
  else()
    message (STATUS "Searching for Ruby - found version ${RUBY_VERSION}.")
  endif()

  ########################################
  # Include python
  find_package(PythonInterp 3 REQUIRED) # change to Python3 when Bionic is EOL
  find_package(PythonLibs REQUIRED)
  if (NOT PYTHONLIBS_FOUND)
    message (STATUS "Searching for Python - not found.")
  else()
    message (STATUS "Searching for Python - found version ${PYTHONLIBS_VERSION_STRING}.")
  endif()
endif()

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
  set_source_files_properties(python.i PROPERTIES SWIG_MODULE_NAME "gazebo")
  set(SWIG_PY_LIB pygazebo)
  set(SWIG_PY_LIB_OUTPUT gazebo)

  set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}/lib/python")
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    SWIG_ADD_LIBRARY(${SWIG_PY_LIB} LANGUAGE python SOURCES python.i)
  else()
    SWIG_ADD_MODULE(${SWIG_PY_LIB} python python.i)
  endif()

  SWIG_LINK_LIBRARIES(${SWIG_PY_LIB}
    ${PYTHON_LIBRARIES}
    ignition-gazebo${PROJECT_VERSION_MAJOR}
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
  install(FILES ${CMAKE_BINARY_DIR}/lib/python/gazebo.py DESTINATION ${IGN_PYTHON_INSTALL_PATH})
endif()
