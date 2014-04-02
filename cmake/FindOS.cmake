# Check the OS type.

# CMake does not distinguish Linux from other Unices.
STRING (REGEX MATCH "Linux" PLAYER_OS_LINUX ${CMAKE_SYSTEM_NAME})
# Nor *BSD
STRING (REGEX MATCH "BSD" PLAYER_OS_BSD ${CMAKE_SYSTEM_NAME})
# Or Solaris. I'm seeing a trend, here
STRING (REGEX MATCH "SunOS" PLAYER_OS_SOLARIS ${CMAKE_SYSTEM_NAME})

# Windows is easy (for once)
IF (WIN32)
    SET (PLAYER_OS_WIN TRUE BOOL INTERNAL)
ENDIF (WIN32)

# Check if it's an Apple OS
IF (APPLE)
    # Check if it's OS X or another MacOS (that's got to be pretty unlikely)
    STRING (REGEX MATCH "Darwin" PLAYER_OS_OSX ${CMAKE_SYSTEM_NAME})
    IF (NOT PLAYER_OS_OSX)
        SET (PLAYER_OS_MACOS TRUE BOOL INTERNAL)
    ENDIF (NOT PLAYER_OS_OSX)
ENDIF (APPLE)

# QNX
IF (QNXNTO)
    SET (PLAYER_OS_QNX TRUE BOOL INTERNAL)
ENDIF (QNXNTO)

IF (PLAYER_OS_LINUX)
    MESSAGE (STATUS "Operating system is Linux")
ELSEIF (PLAYER_OS_BSD)
    MESSAGE (STATUS "Operating system is BSD")
ELSEIF (PLAYER_OS_WIN)
    MESSAGE (STATUS "Operating system is Windows")
ELSEIF (PLAYER_OS_OSX)
    MESSAGE (STATUS "Operating system is Apple MacOS X")
ELSEIF (PLAYER_OS_MACOS)
    MESSAGE (STATUS "Operating system is Apple MacOS (not OS X)")
ELSEIF (PLAYER_OS_QNX)
    MESSAGE (STATUS "Operating system is QNX")
ELSEIF (PLAYER_OS_SOLARIS)
    MESSAGE (STATUS "Operating system is Solaris")
ELSE (PLAYER_OS_LINUX)
    MESSAGE (STATUS "Operating system is generic Unix")
ENDIF (PLAYER_OS_LINUX)

#################################################
# Check for non-case-sensitive filesystems
execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/case_sensitive_filesystem
                RESULT_VARIABLE FILESYSTEM_CASE_SENSITIVE_RETURN)
if (${FILESYSTEM_CASE_SENSITIVE_RETURN} EQUAL 0)
  set(FILESYSTEM_CASE_SENSITIVE TRUE)
else()
  set(FILESYSTEM_CASE_SENSITIVE FALSE)
endif()
