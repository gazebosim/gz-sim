gz_add_system(shader-param
  SOURCES
    ShaderParam.cc
  PUBLIC_LINK_LIBS
    ${rendering_target}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
)
