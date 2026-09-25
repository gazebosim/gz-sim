gz_add_system(shader-param
  SOURCES
    ShaderParam.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    gz-common::gz-common
    gz-rendering::gz-rendering
)
