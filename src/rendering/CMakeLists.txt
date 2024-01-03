set (rendering_comp_sources
  MarkerManager.cc
  RenderUtil.cc
  SceneManager.cc
  WrenchVisualizer.cc
)

if (MSVC)
  # Warning #4251 is the "dll-interface" warning that tells you when types used
  # by a class are not being exported. These generated source files have private
  # members that don't get exported, so they trigger this warning. However, the
  # warning is not important since those members do not need to be interfaced
  # with.
  set_source_files_properties(${rendering_comp_sources}
      COMPILE_FLAGS "/wd4251 /wd4146")
endif()

gz_add_component(rendering
  SOURCES ${rendering_comp_sources}
  GET_TARGET_NAME rendering_target)

target_link_libraries(${rendering_target}
  PUBLIC
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::register
)

install(TARGETS ${rendering_target} DESTINATION ${GZ_LIB_INSTALL_DIR})

set(rendering_target ${rendering_target} PARENT_SCOPE)
