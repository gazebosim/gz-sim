gz_add_gui_plugin(ResourceSpawner
  SOURCES ResourceSpawner.cc
  QT_HEADERS ResourceSpawner.hh
  TEST_SOURCES ResourceSpawner_TEST.cc
  PRIVATE_LINK_LIBS
    gz-utils::gz-utils
)
