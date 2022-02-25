if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()


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
  endif()
else()
  # If not a system installation, respect local paths
  set(IGN_PYTHON_INSTALL_PATH ${IGN_LIB_INSTALL_DIR}/python)
endif()

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${IGN_PYTHON_INSTALL_PATH}/ignition"
  )
endfunction()

pybind11_add_module(gazebo SHARED
  src/ignition/gazebo/_ignition_gazebo_pybind11.cc
  src/ignition/gazebo/EntityComponentManager.cc
  src/ignition/gazebo/EventManager.cc
  src/ignition/gazebo/TestFixture.cc
  src/ignition/gazebo/Server.cc
  src/ignition/gazebo/ServerConfig.cc
  src/ignition/gazebo/UpdateInfo.cc
  src/ignition/gazebo/Util.cc
  src/ignition/gazebo/World.cc
)

target_link_libraries(gazebo PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  sdformat${SDF_VER}::sdformat${SDF_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

# TODO(ahcorde): Move this module to ign-common
pybind11_add_module(common SHARED
  src/ignition/common/_ignition_common_pybind11.cc
  src/ignition/common/Console.cc
)

target_link_libraries(common PRIVATE
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

# TODO(ahcorde): Move this module to sdformat
pybind11_add_module(sdformat SHARED
  src/ignition/sdformat/_sdformat_pybind11.cc
  src/ignition/sdformat/Element.cc
)

target_link_libraries(sdformat PRIVATE
	sdformat${SDF_VER}::sdformat${SDF_VER}
)

install(TARGETS sdformat
	DESTINATION "${IGN_PYTHON_INSTALL_PATH}"
)

configure_build_install_location(gazebo)
configure_build_install_location(common)

if (BUILD_TESTING)
  set(python_tests
    testFixture_TEST
  )

  foreach (test ${python_tests})
    add_test(NAME ${test}.py COMMAND
      "${PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/python/test/${test}.py")

    set(_env_vars)
    list(APPEND _env_vars "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/python/")
    list(APPEND _env_vars "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}:$ENV{LD_LIBRARY_PATH}")
    set_tests_properties(${test}.py PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endforeach()
endif()
