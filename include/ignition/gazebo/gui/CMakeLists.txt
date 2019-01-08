set (CMAKE_AUTOMOC ON)

set (gui_headers
  TmpIface.hh
)

QT5_WRAP_CPP(headers_MOC ${gui_headers})
QT5_ADD_RESOURCES(resources_RCC gazebo.qrc)

include_directories(SYSTEM
  ${IGNITION-COMMON_INCLUDE_DIRS}
  ${IGNITION-GUI_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
  ${IGNITION-MSGS_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickControls2_INCLUDE_DIRS}
)

link_directories(
  ${IGNITION-COMMON_LIBRARY_DIRS}
  ${IGNITION-GUI_LIBRARY_DIRS}
  ${IGNITION-MATH_LIBRARY_DIRS}
  ${IGNITION-MSGS_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
)

set(gui_lib ign-gazebo-gui-lib)

# TODO(anyone) Creating a separate library now, maybe create a component?
add_library(${gui_lib}
  ${gui_sources}
  ${headers_MOC}
  ${resources_RCC}
)

target_link_libraries(${gui_lib}
  PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gui${IGN_GUI_VER}::ignition-gui${IGN_GUI_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
)

add_dependencies(${PROJECT_LIBRARY_TARGET_NAME}
  ${gui_lib}
)

set_target_properties(${gui_lib} PROPERTIES VERSION ${PROJECT_VERSION_FULL})
install (
  TARGETS ${gui_lib}
  EXPORT ${PROJECT_LIBRARY_TARGET_NAME}-all
  DESTINATION ${IGN_LIB_INSTALL_DIR}
  COMPONENT shlib
)

install (FILES gui.config DESTINATION
  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/${IGN_DESIGNATION}${PROJECT_VERSION_MAJOR}/gui)
