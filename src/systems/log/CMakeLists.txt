gz_add_system(log
  SOURCES
    LogRecord.cc
    LogPlayback.cc
  PUBLIC_LINK_LIBS
    ignition-transport${IGN_TRANSPORT_VER}::log
)
