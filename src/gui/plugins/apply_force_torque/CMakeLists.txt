gz_add_gui_plugin(ApplyForceTorque
  SOURCES ApplyForceTorque.cc
  QT_HEADERS ApplyForceTorque.hh
  PUBLIC_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
)
