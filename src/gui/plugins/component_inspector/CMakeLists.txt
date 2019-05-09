set (qt_headers_local
  ComponentInspector
)

# Plugin shared libraries
foreach (src ${qt_headers_local})
  QT5_WRAP_CPP(${src}_MOC ${src}.hh)
  QT5_ADD_RESOURCES(${src}_RCC ${src}.qrc)
  add_library(${src} SHARED
    ${src}.cc
    ${${src}_MOC}
    ${${src}_RCC}
  )
  target_link_libraries(${src}
    PUBLIC
      ${PROJECT_LIBRARY_TARGET_NAME}
      TINYXML2::TINYXML2
    PRIVATE
      ignition-plugin${IGN_PLUGIN_VER}::register
  )
  install (TARGETS ${src} DESTINATION ${IGNITION_GAZEBO_GUI_PLUGIN_INSTALL_DIR})
endforeach (src ${qt_headers_local})
