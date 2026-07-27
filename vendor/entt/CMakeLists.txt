#--------------------------------------
# Find or fetch EnTT for entity-component-system storage
include(FetchContent)
FetchContent_Declare(
  entt
  GIT_REPOSITORY https://github.com/skypjack/entt.git
  GIT_TAG        v3.16.0
  GIT_SHALLOW    TRUE
  PATCH_COMMAND  git checkout -q . && git apply ${CMAKE_CURRENT_SOURCE_DIR}/entt.patch
)
FetchContent_MakeAvailable(entt)
set(ENTT_INCLUDE_DIR "${entt_SOURCE_DIR}/src" PARENT_SCOPE)

# Install EnTT headers into the include directory
if(entt_SOURCE_DIR)
  install(DIRECTORY ${entt_SOURCE_DIR}/src/entt DESTINATION ${GZ_INCLUDE_INSTALL_DIR})
endif()
