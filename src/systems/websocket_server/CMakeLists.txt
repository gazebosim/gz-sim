file (READ combined.proto WEBSOCKETSERVER_MESSAGE_DEFINITIONS)
configure_file("MessageDefinitions.hh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/MessageDefinitions.hh" @ONLY)
# Add a dependency on binary source dir so that you can change the
# combinded.proto file and `make` will rebuild the plugin.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS combined.proto)

#if (WIN32)
#  set(websockets_lib websockets)
#else()
#  set(websockets_lib websockets_shared)
#endif()

message("libwebsockets ============ ${websockets_shared}")
message("libwebsockets_dep_libs  ============ ${LIBWEBSOCKETS_DEP_LIBS}")

gz_add_system(websocket-server
  SOURCES
    WebsocketServer.cc
  PUBLIC_LINK_LIBS
    websockets_shared
    ${LIBWEBSOCKETS_DEP_LIBS}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-common${GZ_COMMON_VER}::graphics
    gz-msgs${GZ_MSGS_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::core
    gz-transport${GZ_TRANSPORT_VER}::core
  PRIVATE_INCLUDE_DIRS
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
)
