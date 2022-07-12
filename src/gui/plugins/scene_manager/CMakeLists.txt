gz_add_gui_plugin(GzSceneManager
  SOURCES GzSceneManager.cc
  QT_HEADERS GzSceneManager.hh
  PRIVATE_LINK_LIBS
    ${PROJECT_LIBRARY_TARGET_NAME}-rendering
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
)
