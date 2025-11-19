gz_add_system(lock-step
  SOURCES
  LockStep.cc
  PUBLIC_LINK_LIBS
    gz-transport::gz-transport
)

set(runtime_executable lockstep-runtime)
add_executable(${runtime_executable}
  runtime_main.cc
  RuntimeConfig.cc
  LockStepRuntime.cc)
target_link_libraries(${runtime_executable}
  PRIVATE
  Backward::Object
  gz-common::gz-common
  gz-utils::cli
  gz-sim
)

install(
  TARGETS ${runtime_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)
