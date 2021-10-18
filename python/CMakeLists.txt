find_package(pybind11 REQUIRED)

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()

function(clean_windows_flags target)
  # Hack to avoid pybind11 issue.
  #
  # TODO(ivanpauno):
  # This can be deleted when we update `pybind11_vendor` to a version including
  # https://github.com/pybind/pybind11/pull/2590.
  #
  # They are enabling /LTCG on Windows to reduce binary size,
  # but that doesn't play well with MSVC incremental linking (default for Debug/RelWithDebInfo).
  #
  # See:
  # - https://docs.microsoft.com/en-us/cpp/build/reference/incremental-link-incrementally?view=vs-2019
  # - https://docs.microsoft.com/en-us/cpp/build/reference/ltcg-link-time-code-generation?view=vs-2019

  if(MSVC AND "${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    get_target_property(target_link_libraries ${target} LINK_LIBRARIES)
    list(REMOVE_ITEM target_link_libraries "$<$<NOT:$<CONFIG:Debug>>:-LTCG>")
    set_target_properties(${target} PROPERTIES LINK_LIBRARIES "${target_link_libraries}")

    get_target_property(target_compile_options ${target} COMPILE_OPTIONS)
    list(REMOVE_ITEM target_compile_options "$<$<NOT:$<CONFIG:Debug>>:/GL>")
    set_target_properties(${target} PROPERTIES COMPILE_OPTIONS "${target_compile_options}")
  endif()
endfunction()

# # enables using the Python extensions from the build space for testing
# file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_rclpy/__init__.py" "")

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install into test_rclpy folder in build space for unit tests to import
  set_target_properties(${_library_name} PROPERTIES
    # Use generator expression to avoid prepending a build type specific directory on Windows
    LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test_rclpy>
    RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/test_rclpy>)

  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endfunction()

# Split from main extension and converted to pybind11
pybind11_add_module(ignition_gazebo SHARED
  src/ignition/gazebo/_ignition_gazebo_pybind11.cc
  src/ignition/utils/destroyable.cc
  src/ignition/gazebo/entity_component_manager
  src/ignition/gazebo/event_manager.cc
  src/ignition/gazebo/helper_system.cc
  src/ignition/gazebo/server_config.cc
  src/ignition/gazebo/server.cc
  src/ignition/gazebo/update_info.cc
  src/ignition/gazebo/world.cc
)

target_link_libraries(ignition_gazebo PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  sdformat${SDF_VER}::sdformat${SDF_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

# Split from main extension and converted to pybind11
pybind11_add_module(ignition_transport SHARED
  src/ignition/transport/_ignition_transport_pybin11.cc
  src/ignition/utils/destroyable.cc
  src/ignition/transport/node.cc
  src/ignition/msgs/pose_v.cc
)

target_link_libraries(ignition_transport PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  sdformat${SDF_VER}::sdformat${SDF_VER}
)

# Split from main extension and converted to pybind11
pybind11_add_module(ignition_common SHARED
  src/ignition/common/_ignition_common_pybind11.cc
  src/ignition/common/console.cc
)

target_link_libraries(ignition_common PRIVATE
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)

# Split from main extension and converted to pybind11
pybind11_add_module(ignition_msgs SHARED
  src/ignition/msgs/_ignition_msgs_pybind11.cc
  src/ignition/utils/destroyable.cc
  src/ignition/msgs/pose_v.cc
)

target_link_libraries(ignition_msgs PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  sdformat${SDF_VER}::sdformat${SDF_VER}
)

add_subdirectory(swig)
