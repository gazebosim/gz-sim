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

# TODO: someone with knowledge of gz-plugin please resolve:
# TestSystem.obj : error LNK2001: unresolved external symbol GzPluginHook
if(NOT WIN32)
  set (test_plugins
    ${test_plugins}
    TestSystem
  )
endif()

# Plugin shared libraries
if(BUILD_TESTING)
  foreach (src ${test_plugins})
    add_library(${src} SHARED ${src}.cc)
    target_link_libraries(${src}
      PRIVATE
        gz-plugin${GZ_PLUGIN_VER}::register
        gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
        gz-sim${PROJECT_VERSION_MAJOR}
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
