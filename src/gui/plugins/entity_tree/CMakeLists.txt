gz_add_gui_plugin(EntityTree
  SOURCES EntityTree.cc
  QT_HEADERS EntityTree.hh
  TEST_SOURCES EntityTree_TEST.cc
  PRIVATE_LINK_LIBS
    gz-common::graphics
)
