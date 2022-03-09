gz_add_system(comms-broker
  SOURCES
  BrokerPlugin.cc
  Broker.cc
  PUBLIC_LINK_LIBS
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
