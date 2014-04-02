#
# Based on work of Emmanuel Roullit <emmanuel@netsniff-ng.org>
# Copyright 2009, 2012 Emmanuel Roullit.
# Subject to the GPL, version 2.
#
MACRO(ADD_MANPAGE_TARGET)
  # It is not possible add a dependency to target 'install'
  # Run hard-coded 'make man' when 'make install' is invoked
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND make man)")
  ADD_CUSTOM_TARGET(man)
ENDMACRO(ADD_MANPAGE_TARGET)

FIND_PROGRAM(RONN ronn)
FIND_PROGRAM(GZIP gzip)

IF (NOT RONN OR NOT GZIP)
  IF (NOT RONN)
    BUILD_WARNING ("ronn not found, manpages won't be generated")
  ENDIF(NOT RONN)
  IF (NOT GZIP)
    BUILD_WARNING ("gzip not found, manpages won't be generated")
  ENDIF(NOT GZIP)
  # empty macro
  MACRO(manpage MANFILE)
  ENDMACRO(manpage)
  SET (MANPAGES_SUPPORT FALSE)
ELSE (NOT RONN OR NOT GZIP)
  MESSAGE (STATUS "Looking for ronn to generate manpages - found")
  SET (MANPAGES_SUPPORT TRUE)

  MACRO(manpage RONNFILE SECTION)
    SET(RONNFILE_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${RONNFILE})

    ADD_CUSTOM_COMMAND(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}
      DEPENDS ${RONNFILE}
      COMMAND ${RONN}
         ARGS -r --pipe ${RONNFILE_FULL_PATH}.${SECTION}.ronn
         > ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}
    )

    ADD_CUSTOM_COMMAND(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}.gz
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}
      COMMAND ${GZIP} -c ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}
        > ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}.gz
    )

    SET(MANPAGE_TARGET "man-${RONNFILE}")

    ADD_CUSTOM_TARGET(${MANPAGE_TARGET} DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}.gz)
    ADD_DEPENDENCIES(man ${MANPAGE_TARGET})

    INSTALL(
      FILES ${CMAKE_CURRENT_BINARY_DIR}/${RONNFILE}.${SECTION}.gz
      DESTINATION share/man/man${SECTION}
    )
  ENDMACRO(manpage RONNFILE SECTION)
ENDIF(NOT RONN OR NOT GZIP)
