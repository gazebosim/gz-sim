if (WIN32)
  add_definitions(-DBUILDING_DLL)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}
)

link_directories(
)

set (plugins
  EventTriggerSystem
  TestSystem
  MockSystem
  Move3dSystem
)

set(IGNITION_GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/plugins
)
# Plugin shared libraries
foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  set_property(TARGET ${src} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${src}
    PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    PRIVATE ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    PRIVATE ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  )
  install(TARGETS ${src} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
endforeach (src)
