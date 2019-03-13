set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)

#################################################
# gz_add_system(<system>
#              SOURCES <sources>
#              [PUBLIC_LINK_LIBS <libraries...>]
#              [PRIVATE_LINK_LIBS <libraries...>]
#
# Add a system component to Ignition Gazebo.
#
# <system> Required. Name of the system.
#
# SOURCES: Required. Specify the source files for the system.
#
# [PUBLIC_LINK_LIBS]: Specify a list of libraries to be publicly linked.
#
# [PRIVATE_LINK_LIBS]: Specify a list of libraries to be privately linked.
#
# [PRIVATE_COMPILE_DEFS]: Specify a list of private compile definitions to add to target.
#
function(gz_add_system system_name)

  set(options)
  set(oneValueArgs)
  set(multiValueArgs SOURCES PUBLIC_LINK_LIBS PRIVATE_LINK_LIBS PRIVATE_COMPILE_DEFS)

  cmake_parse_arguments(gz_add_system "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(gz_add_system_SOURCES)
    set(sources ${gz_add_system_SOURCES})
  else()
    message(FATAL_ERROR "You must specify SOURCES for gz_add_system(~)!")
  endif()

  set(system_name ${system_name}-system)

  ign_add_component(${system_name}
    SOURCES ${sources}
    GET_TARGET_NAME system_target
    CXX_STANDARD 17)

  target_link_libraries(${system_target}
    PUBLIC
      ${gz_add_system_PUBLIC_LINK_LIBS}
      ${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE
      ${gz_add_system_PRIVATE_LINK_LIBS}
      ignition-plugin${IGN_PLUGIN_VER}::register
  )

  if(gz_add_system_PRIVATE_COMPILE_DEFS)
    target_compile_definitions(${system_target}
      PRIVATE
        ${gz_add_system_PRIVATE_COMPILE_DEFS}
    )
  endif()

  install(TARGETS ${system_target} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

  # The library created by `ign_add_component` includes the ign-gazebo version
  # (i.e. libignition-gazebo1-name-system.so), but for portability of SDF
  # files, we also install an unversioned symlink.
  INSTALL(CODE "execute_process( \
    COMMAND ${CMAKE_COMMAND} -E create_symlink \
    ${CMAKE_SHARED_LIBRARY_PREFIX}${system_target}${CMAKE_SHARED_LIBRARY_SUFFIX} \
    ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${system_name}${CMAKE_SHARED_LIBRARY_SUFFIX} \
    )"
  )
endfunction()

add_subdirectory(altimeter)
add_subdirectory(contact)
add_subdirectory(diff_drive)
add_subdirectory(imu)
add_subdirectory(log)
add_subdirectory(logical_camera)
add_subdirectory(magnetometer)
add_subdirectory(physics)
add_subdirectory(pose_publisher)
add_subdirectory(sensors)
add_subdirectory(user_commands)
add_subdirectory(scene_broadcaster)
add_subdirectory(touch_plugin)
