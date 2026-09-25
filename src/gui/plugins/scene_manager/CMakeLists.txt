gz_add_gui_plugin(GzSceneManager
  SOURCES GzSceneManager.cc
  QT_HEADERS GzSceneManager.hh
  PRIVATE_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
    gz-utils::gz-utils
)
