if (WIN32)
  add_definitions(-DBUILDING_DLL)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}
)

link_directories(
)

set (plugins
  EventTriggerSystem
  TestSystem
  MockSystem
)

# Plugin shared libraries
foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  set_property(TARGET ${src} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${src}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  )
endforeach (src)
