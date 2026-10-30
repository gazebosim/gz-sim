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

# A physics engine plugin that records the worlds it is asked to construct, and
# the library with the record, for tests to link. The plugin must not be linked
# by tests: a test that links a plugin library cannot load the system plugins.
if(BUILD_TESTING)
  add_library(MockPhysicsEngineRecorder SHARED MockPhysicsEngineRecorder.cc)
  # The test calls the functions of this library. Without export macros, make
  # the symbols visible, as the project builds with hidden visibility.
  set_target_properties(MockPhysicsEngineRecorder PROPERTIES
    CXX_VISIBILITY_PRESET default
    WINDOWS_EXPORT_ALL_SYMBOLS ON)
  target_link_libraries(MockPhysicsEngineRecorder
    PRIVATE
      gz-math::gz-math
  )

  add_library(MockPhysicsEngine MODULE MockPhysicsEngine.cc)
  target_link_libraries(MockPhysicsEngine
    PRIVATE
      MockPhysicsEngineRecorder
      gz-plugin::register
      gz-physics::gz-physics
      gz-math::gz-math
      sdformat::sdformat
  )

  if (MSVC)
    set_source_files_properties(MockPhysicsEngine.cc
      MockPhysicsEngineRecorder.cc COMPILE_FLAGS "/wd4251 /wd4146")
  endif()
endif()
