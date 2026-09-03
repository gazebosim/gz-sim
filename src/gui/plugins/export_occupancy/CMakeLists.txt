gz_add_gui_plugin(ExportOccupancy
  SOURCES ExportOccupancy.cc
  QT_HEADERS ExportOccupancy.hh
  PRIVATE_LINK_LIBS
    gz-common::gz-common
    gz-common::io
    gz-math::gz-math
)
