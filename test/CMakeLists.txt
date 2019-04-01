include_directories (
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
  ${PROJECT_BINARY_DIR}/include/
)

configure_file (test_config.hh.in ${PROJECT_BINARY_DIR}/include/ignition/gazebo/test_config.hh)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")

# Build gbenchmark
file(GLOB
  GBENCHMARK_SOURCE_FILES
    gbenchmark/src/*.cc
    gbenchmark/src/*.h
    gbenchmark/include/benchmark/*.h
)

add_library(gbenchmark ${GBENCHMARK_SOURCE_FILES})
target_include_directories(gbenchmark PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gbenchmark/include>
)
set_property(TARGET gbenchmark PROPERTY CXX_STANDARD 17)
target_compile_options(gbenchmark PRIVATE "-Wno-switch-default"
  "-Wno-unused-but-set-variable"
  "-Wno-switch-default"
  "-Wno-deprecated-declarations"
)

if(UNIX)
  target_link_libraries(gbenchmark pthread)
endif()

find_library(LIBRT rt)
if(LIBRT)
  target_link_libraries(gbenchmark ${LIBRT})
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  target_link_libraries(gbenchmark Shlwapi)
endif()

add_library(gbenchmark_main
  gbenchmark/src/benchmark_main.cc)
target_include_directories(gbenchmark_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/gbenchmark/include>
)
set_property(TARGET gbenchmark_main PROPERTY CXX_STANDARD 17)
target_compile_options(gbenchmark_main
  PUBLIC
  "-Wno-switch-default"
  PRIVATE
  "-Wno-unused-but-set-variable"
  "-Wno-deprecated-declarations"
)
target_link_libraries(gbenchmark_main gbenchmark)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

################################# FindDRI support #############################
# Check for existance of glxinfo application
# Check for existance of support for pyopengl
function(FindDRI)

message(STATUS "Looking for display capabilities")

if((DEFINED FORCE_GRAPHIC_TESTS_COMPILATION)
  AND (${FORCE_GRAPHIC_TESTS_COMPILATION}))
  set(VALID_DISPLAY TRUE PARENT_SCOPE)
  set(VALID_DRI_DISPLAY TRUE PARENT_SCOPE)
  message(STATUS " + Force requested. All capabilities on without checking")
  return()
endif()

set(VALID_DISPLAY FALSE)
set(VALID_DRI_DISPLAY FALSE)
set(CHECKER_ERROR "(no glxinfo or pyopengl)")

if((DEFINED ENV{DISPLAY}) AND NOT ("$ENV{DISPLAY}" STREQUAL ""))
  find_program(XWININFO xwininfo)
  if (XWININFO)
    execute_process(
      COMMAND xwininfo -root
      RESULT_VARIABLE DISPLAY_FAIL_RESULT
      ERROR_QUIET OUTPUT_QUIET
    )
  else()
    message(STATUS "Could not find 'xwininfo', which is needed for FindDRI. Please install the package 'x11-utils'")
    return()
  endif()

  if(NOT DISPLAY_FAIL_RESULT)
    message(STATUS " + found a display available ($DISPLAY is set)")
    set(VALID_DISPLAY TRUE)

    # Continue check for DRI support in the display Try to run glxinfo. If not
    # found, variable will be empty
    find_program(GLXINFO glxinfo)

    # If not display found, it will throw an error
    # Another grep pattern: "direct rendering:[[:space:]]*Yes[[:space:]]*"
    if(GLXINFO)
      execute_process(
        COMMAND glxinfo
        COMMAND grep "direct rendering:[[:space:]]*Yes[[:space:]]*"
        ERROR_QUIET
        OUTPUT_VARIABLE GLX
      )

      if(GLX)
        message(STATUS " + found a valid dri display (glxinfo)")
        set(VALID_DRI_DISPLAY TRUE)
      else()
        set(CHECKER_ERROR "using glxinfo")
      endif()
    else()
      message(STATUS
        "Could not find glxinfo. Trying with 'gl-test.py' which may not work."
        "If 'gl-test.py' fails to find DRI, try installng the package "
        "'mesa-utils' for glxinfo")
      execute_process(
        # RESULT_VARIABLE is store in a FAIL variable since the command
        # returns 0 if ok and 1 if error (inverse than cmake IF)
        COMMAND ${PROJECT_SOURCE_DIR}/tools/gl-test.py
        RESULT_VARIABLE GL_FAIL_RESULT
        ERROR_VARIABLE GL_ERROR
        OUTPUT_QUIET
      )

      if(NOT GL_FAIL_RESULT)
        message(STATUS " + found a valid dri display (pyopengl)")
        set(VALID_DRI_DISPLAY TRUE)
      elseif(${GL_ERROR})
        # Check error string: no python module means no pyopengl
        string(
          FIND ${GL_ERROR} "ImportError: No module named OpenGL.GLUT" ERROR_POS
        )
        # -1 will imply pyopengl is present but real DRI test fails
        if("${ERROR_POS}" STREQUAL "-1")
          set(CHECKER_ERROR "using pyopengl")
        endif()
      endif()
    endif()
  endif()
endif()

if(NOT VALID_DISPLAY)
  message(STATUS " ! valid display not found")
endif()

if(NOT VALID_DRI_DISPLAY)
  message(STATUS " ! valid dri display not found ${CHECKER_ERROR}")
endif()

# Set variables to parent scope
set(VALID_DISPLAY ${VALID_DISPLAY} PARENT_SCOPE)
set(VALID_DRI_DISPLAY ${VALID_DRI_DISPLAY} PARENT_SCOPE)

endfunction()

############################## End FindDRI support #############################

FindDRI()

add_subdirectory(benchmark)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(plugins)
add_subdirectory(regression)
