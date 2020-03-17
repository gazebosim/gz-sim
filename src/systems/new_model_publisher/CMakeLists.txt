gz_add_system(new-model-publisher
  SOURCES
    NewModelPublisher.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
