find_package(Qt5Test REQUIRED)

set (qt_headers
  GuiRelay.hh
)

QT5_WRAP_CPP(headers_MOC ${qt_headers})

add_library(${PROJECT_NAME}_test_helpers SHARED
  ${headers_MOC}
)
target_link_libraries(${PROJECT_NAME}_test_helpers
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
)
