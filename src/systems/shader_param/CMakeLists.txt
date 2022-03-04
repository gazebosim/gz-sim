gz_add_system(shader-param
  SOURCES
    ShaderParam.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
)
