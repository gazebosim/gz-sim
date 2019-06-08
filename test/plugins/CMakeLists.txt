if (WIN32)
  add_definitions(-DBUILDING_DLL)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}
)

link_directories(
)

set (test_plugins
  EventTriggerSystem
  TestModelSystem
  TestSensorSystem
  TestSystem
  TestWorldSystem
  MockSystem
  Null
)

# Plugin shared libraries
if(BUILD_TESTING)
  foreach (src ${test_plugins})
    add_library(${src} SHARED ${src}.cc)
    set_property(TARGET ${src} PROPERTY CXX_STANDARD 17)
    target_link_libraries(${src}
      PRIVATE
        ignition-plugin${IGN_PLUGIN_VER}::register
        ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
        ignition-gazebo${PROJECT_VERSION_MAJOR}
    )
  endforeach (src)
endif()
