# Create an IgnGazebo module
set(module_name IgnGazebo)

# Add EntityContextMenu QML plugin to IgnGazebo module
gz_add_gui_library(EntityContextMenu
  SOURCES EntityContextMenu.cc
  QT_HEADERS EntityContextMenu.hh
)
install (TARGETS EntityContextMenu DESTINATION ${IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR}/${module_name})

install (FILES qmldir DESTINATION ${IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR}/${module_name})

