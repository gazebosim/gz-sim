set (gui_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/GuiRunner.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/TmpIface.cc
  PARENT_SCOPE
)

add_subdirectory(plugins)
