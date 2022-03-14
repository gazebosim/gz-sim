gz_add_system(perfect-comms-model
  SOURCES
  PerfectCommsModel.cc
  ../MsgManager.cc
  ../Broker.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
