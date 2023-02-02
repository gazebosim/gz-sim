gz_add_gui_plugin(GlobalIlluminationVct
  SOURCES GlobalIlluminationVct.cc GlobalIlluminationVct.qml GlobalIlluminationVct.qrc
  QT_HEADERS GlobalIlluminationVct.hh
  PRIVATE_LINK_LIBS
    ${GZ-RENDERING_LIBRARIES}
)
