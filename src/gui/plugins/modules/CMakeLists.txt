# Create a GzSim module
set(module_name GzSim)

# Add EntityContextMenu QML plugin to GzSim module
gz_add_gui_library(EntityContextMenu
  SOURCES EntityContextMenu.cc
  QT_HEADERS EntityContextMenu.hh
)

install (TARGETS EntityContextMenu DESTINATION ${GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR}/${module_name})
install (FILES qmldir DESTINATION ${GZ_SIM_GUI_PLUGIN_RELATIVE_INSTALL_DIR}/${module_name})
