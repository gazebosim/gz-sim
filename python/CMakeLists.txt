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
    string(REPLACE "site-packages" "dist-packages" GZ_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
  else()
    # custom cmake command is returning dist-packages
    string(REPLACE "dist-packages" "site-packages" GZ_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
  endif()
else()
  # If not a system installation, respect local paths
  set(GZ_PYTHON_INSTALL_PATH ${GZ_LIB_INSTALL_DIR}/python)
endif()

set(GZ_PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
set(GZ_PYTHON_INSTALL_PATH "${GZ_PYTHON_INSTALL_PATH}/gz")

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${GZ_PYTHON_INSTALL_PATH}"
  )
endfunction()

set(BINDINGS_MODULE_NAME "sim${PROJECT_VERSION_MAJOR}")
pybind11_add_module(${BINDINGS_MODULE_NAME} MODULE
  src/gz/sim/_gz_sim_pybind11.cc
  src/gz/sim/Actor.cc
  src/gz/sim/EntityComponentManager.cc
  src/gz/sim/EventManager.cc
  src/gz/sim/Joint.cc
  src/gz/sim/Light.cc
  src/gz/sim/Link.cc
  src/gz/sim/Model.cc
  src/gz/sim/TestFixture.cc
  src/gz/sim/Sensor.cc
  src/gz/sim/Server.cc
  src/gz/sim/ServerConfig.cc
  src/gz/sim/UpdateInfo.cc
  src/gz/sim/Util.cc
  src/gz/sim/World.cc
)

target_link_libraries(${BINDINGS_MODULE_NAME} PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

set_target_properties(${BINDINGS_MODULE_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden")

target_compile_definitions(${BINDINGS_MODULE_NAME} PRIVATE
  BINDINGS_MODULE_NAME=${BINDINGS_MODULE_NAME})

# TODO(ahcorde): Move this module to gz-common

set(GZ_COMMON_BINDINGS_MODULE_NAME "common${GZ_COMMON_VER}")
pybind11_add_module(${GZ_COMMON_BINDINGS_MODULE_NAME} MODULE
  src/gz/common/_gz_common_pybind11.cc
  src/gz/common/Console.cc
)

target_compile_definitions(${GZ_COMMON_BINDINGS_MODULE_NAME} PRIVATE
  BINDINGS_MODULE_NAME=${GZ_COMMON_BINDINGS_MODULE_NAME})

target_link_libraries(${GZ_COMMON_BINDINGS_MODULE_NAME} PRIVATE
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)

configure_build_install_location(${BINDINGS_MODULE_NAME})
configure_build_install_location(${GZ_COMMON_BINDINGS_MODULE_NAME})

if (BUILD_TESTING)
  set(python_tests
    actor_TEST
    joint_TEST
    light_TEST
    link_TEST
    model_TEST
    sensor_TEST
    testFixture_TEST
    world_TEST
  )

  execute_process(COMMAND "${GZ_PYTHON_EXECUTABLE}" -m pytest --version
    OUTPUT_VARIABLE PYTEST_output
    ERROR_VARIABLE  PYTEST_error
    RESULT_VARIABLE PYTEST_result)
  if(${PYTEST_result} EQUAL 0)
    set(pytest_FOUND TRUE)
  else()
    message("")
    message(WARNING "Pytest package not available: ${PYTEST_error}")
  endif()

  foreach (test ${python_tests})
    if (pytest_FOUND)
      add_test(NAME ${test} COMMAND
        "${GZ_PYTHON_EXECUTABLE}" -m pytest "${CMAKE_SOURCE_DIR}/python/test/${test}.py" --junitxml "${CMAKE_BINARY_DIR}/test_results/UNIT_${test}.xml")
    else()
      add_test(NAME ${test} COMMAND
        "${GZ_PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/python/test/${test}.py")
    endif()

    set(_env_vars)
    list(APPEND _env_vars "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/python/:$ENV{PYTHONPATH}")
    list(APPEND _env_vars "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}:$ENV{LD_LIBRARY_PATH}")
    set_tests_properties(${test} PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endforeach()
endif()
