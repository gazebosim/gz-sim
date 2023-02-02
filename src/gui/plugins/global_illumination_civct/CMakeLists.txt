gz_add_gui_plugin(GlobalIlluminationCiVct
  SOURCES CiVctCascadePrivate.cc CiVctCascadePrivate.qml GlobalIlluminationCiVct.cc GlobalIlluminationCiVct.qml GlobalIlluminationCiVct.qrc
  QT_HEADERS CiVctCascadePrivate.hh GlobalIlluminationCiVct.hh
  PRIVATE_LINK_LIBS
    ${GZ-RENDERING_LIBRARIES}
)
