find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/ignition.in
                 ${CMAKE_BINARY_DIR}/ignition.dox @ONLY)

  add_custom_target(doc
    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/ignition.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/ignition_logo.svg
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()
