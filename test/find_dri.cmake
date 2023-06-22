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
