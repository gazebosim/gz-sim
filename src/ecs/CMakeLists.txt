# Phase 0a: standalone archetype ECS core (gz-sim-ecs).
#
# This library is intentionally decoupled from the main gz-sim library — per
# the Phase 0a design, 0a ships the data structures only. Phase 0b wires the
# ECS core into SimulationRunner.

set(ECS_PUBLIC_HEADERS
  ${PROJECT_SOURCE_DIR}/include/gz/sim/ecs/Entity.hh
  ${PROJECT_SOURCE_DIR}/include/gz/sim/ecs/ComponentTypeRegistry.hh
  ${PROJECT_SOURCE_DIR}/include/gz/sim/ecs/World.hh
  ${PROJECT_SOURCE_DIR}/include/gz/sim/ecs/Query.hh
  ${PROJECT_SOURCE_DIR}/include/gz/sim/ecs/CommandBuffer.hh
)

set(ECS_SOURCES
  ComponentTypeRegistry.cc
  EntityIndex.cc
  Chunk.cc
  Archetype.cc
  ArchetypeGraph.cc
  Query.cc
  CommandBuffer.cc
  World.cc
  ArchetypeBackedECM.cc
)

add_library(gz-sim-ecs STATIC ${ECS_SOURCES})

target_include_directories(gz-sim-ecs
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>  # for generated gz/sim/config.hh
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(gz-sim-ecs PUBLIC cxx_std_20)

# Position-independent so this static lib can eventually be folded into a
# shared gz-sim in Phase 0b.
set_target_properties(gz-sim-ecs PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Phase 0b: when absorbed into libgz-sim via --whole-archive, symbols
# carry forward their visibility. gz-sim compiles with -fvisibility=hidden
# and relies on GZ_SIM_VISIBLE annotations; gz-sim-ecs symbols don't carry
# that annotation and would become hidden in the resulting shared
# library, breaking template instantiations in downstream test/plugin
# TUs. Compile gz-sim-ecs with default visibility so every symbol
# stays exported.
if(NOT WIN32)
  target_compile_options(gz-sim-ecs PRIVATE
    -fvisibility=default -fvisibility-inlines-hidden)
endif()

# gz-common provides hash64() + WorkerPool. Types.hh comes in via the
# gz-sim include tree (header-only dependency in 0a).
target_link_libraries(gz-sim-ecs PUBLIC
  gz-common::gz-common
)

if(UNIX)
  target_link_libraries(gz-sim-ecs PUBLIC pthread)
endif()

if(BUILD_TESTING)
  set(ECS_TEST_SOURCES
    ${PROJECT_SOURCE_DIR}/test/ecs/Entity_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/EntityIndex_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/ComponentTypeRegistry_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/Chunk_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/Archetype_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/World_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/CommandBuffer_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/EachParallel_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/ChangeTracking_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/Fuzz_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/ArchetypeBackedECM_TEST.cc
    ${PROJECT_SOURCE_DIR}/test/ecs/SystemPortParity_TEST.cc
  )

  find_package(GTest QUIET)
  if(GTest_FOUND OR TARGET GTest::gtest_main)
    foreach(test_src ${ECS_TEST_SOURCES})
      get_filename_component(test_name ${test_src} NAME_WE)
      set(test_target UNIT_ECS_${test_name})
      if(EXISTS ${test_src})
        add_executable(${test_target} ${test_src})
        target_link_libraries(${test_target}
          PRIVATE gz-sim-ecs GTest::gtest GTest::gtest_main pthread)
        target_include_directories(${test_target} PRIVATE
          ${CMAKE_CURRENT_SOURCE_DIR})
        add_test(NAME ${test_target} COMMAND ${test_target})
      endif()
    endforeach()
  endif()
endif()

# Archetype-ECS microbenches live under test/benchmark/ alongside the
# rest of the gz-sim benchmarks; they are built when
# GZ_SIM_ARCHETYPE_ECM=ON.
