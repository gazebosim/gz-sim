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
  TestSystem
  TestWorldSystem
  MockSystem
)

# Plugin shared libraries
foreach (src ${test_plugins})
  add_library(${src} SHARED ${src}.cc)
  set_property(TARGET ${src} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${src}
    PRIVATE
      ignition-plugin${IGN_PLUGIN_VER}::register
      ignition-gazebo
  )
endforeach (src)
