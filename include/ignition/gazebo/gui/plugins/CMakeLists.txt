set (qt_headers_local
  EntityTree
)

set (qt_headers_local_rendering
  RenderWindow
)

foreach (src ${qt_headers_local})
  QT5_WRAP_CPP(${src}_MOC ${src}.hh)
  QT5_ADD_RESOURCES(${src}_RCC ${src}.qrc)
  add_library(${src} SHARED
    ${PROJECT_SOURCE_DIR}/src/gui/plugins/${src}.cc
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
  install (TARGETS ${src} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
endforeach (src ${qt_headers_local})


foreach (src ${qt_headers_local_rendering})
  QT5_WRAP_CPP(${src}_MOC ${src}.hh)
  QT5_ADD_RESOURCES(${src}_RCC ${src}.qrc)
  add_library(${src} SHARED
    ${PROJECT_SOURCE_DIR}/src/gui/plugins/${src}.cc
    ${${src}_MOC}
    ${${src}_RCC}
    ${PROJECT_SOURCE_DIR}/src/gui/plugins/Rendering.cc
  )
  target_link_libraries(${src}
    PUBLIC
      ${PROJECT_LIBRARY_TARGET_NAME}
      TINYXML2::TINYXML2
    PRIVATE
      ignition-plugin${IGN_PLUGIN_VER}::register
      ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
  )
  install (TARGETS ${src} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
endforeach (src ${qt_headers_local})


# Plugin shared libraries
#macro (build_plugin sources lib_dep)
##foreach (src ${qt_headers_local})
#
#
#foreach (src ${sources})
#  QT5_WRAP_CPP(${src}_MOC ${src}.hh)
#  QT5_ADD_RESOURCES(${src}_RCC ${src}.qrc)
#  add_library(${src} SHARED
#    ${PROJECT_SOURCE_DIR}/src/gui/plugins/${src}.cc
#    ${${src}_MOC}
#    ${${src}_RCC}
#  )
#  target_link_libraries(${src}
#    PUBLIC
#      ${PROJECT_LIBRARY_TARGET_NAME}
#      TINYXML2::TINYXML2
#    PRIVATE
#      ignition-plugin${IGN_PLUGIN_VER}::register
#      ${lib_dep}
#  )
#  install (TARGETS ${src} DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
##endforeach (src ${qt_headers_local})
#endforeach (src ${sources})
#endmacro()
#
#build_plugin(${qt_headers_local} "")
#build_plugin(${qt_headers_local_rendering}
#  #igntion-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER})
#  igntion-rendering${IGN_RENDERING_VER}::gziface)
