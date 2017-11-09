find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/ignition.in
                 ${CMAKE_BINARY_DIR}/ignition.dox @ONLY)

  configure_file(${CMAKE_SOURCE_DIR}/doc/tutorials.in
                 ${CMAKE_BINARY_DIR}/tutorials.dox @ONLY)

  add_custom_target(doc
    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/ignition.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}

    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/ignition_logo.svg
            ${CMAKE_BINARY_DIR}/doxygen/html

    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/tutorials.dox

    COMMENT "Generating API documentation with Doxygen" VERBATIM)

  install(FILES ${CMAKE_BINARY_DIR}/doc/${PROJECT_NAME_LOWER}.tag.xml 
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ignition/${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}_${PROJECT_MINOR_VERSION})
endif()
