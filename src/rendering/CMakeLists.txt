set (rendering_comp_sources
  RenderUtil.cc
)

ign_add_component(rendering
  SOURCES ${rendering_comp_sources}
  GET_TARGET_NAME rendering_target
  CXX_STANDARD 17)

target_link_libraries(${rendering_target}
  PUBLIC
    ignition-rendering${IGN_RENDERING_VER}::gziface
  PRIVATE
    ignition-plugin${IGN_PLUGIN_VER}::register
)

install(TARGETS ${rendering_target} DESTINATION ${IGN_LIB_INSTALL_DIR})

