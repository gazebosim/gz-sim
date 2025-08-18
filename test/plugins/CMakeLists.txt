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
  TestVisualSystem
  TestWorldSystem
  MockSystem
  Null
)

set (test_plugins
  ${test_plugins}
  TestSystem
)

# Plugin shared libraries
if(BUILD_TESTING)
  foreach (src ${test_plugins})
    add_library(${src} MODULE ${src}.cc)
    target_link_libraries(${src}
      PRIVATE
        gz-common::gz-common
        gz-plugin::register
        gz-transport::gz-transport
        gz-sim
    )

    if (MSVC)
      # Warning #4251 is the "dll-interface" warning that tells you when types used
      # by a class are not being exported. These generated source files have private
      # members that don't get exported, so they trigger this warning. However, the
      # warning is not important since those members do not need to be interfaced
      # with.
      set_source_files_properties(${src}.cc COMPILE_FLAGS "/wd4251 /wd4146")
    endif()
  endforeach (src)
endif()

# Statically linked test plugin
add_library(TestStaticModelSystem STATIC TestStaticModelSystem.cc)
target_link_libraries(TestStaticModelSystem PRIVATE
    gz-common::gz-common
    gz-plugin::register
    gz-transport::gz-transport
    gz-sim)
