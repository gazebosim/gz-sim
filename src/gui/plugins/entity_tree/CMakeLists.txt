gz_add_gui_plugin(EntityTree
  SOURCES EntityTree.cc
  QT_HEADERS EntityTree.hh
  PRIVATE_LINK_LIBS
    gz-common${GZ_COMMON_VER}::graphics
)
