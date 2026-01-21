cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)
project(external_renderer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(gz-rendering REQUIRED)
find_package(FlatBuffers REQUIRED)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

if (NOT APPLE)
  find_package(GLEW REQUIRED)
endif()

# Flatbuffers schema generation
find_program(FLATBUFFERS_FLATC_EXECUTABLE NAMES flatc)

function(build_flatbuffers SCHEMA_FILE GENERATED_PATH)
    get_filename_component(SCHEMA_NAME ${SCHEMA_FILE} NAME_WE)
    set(GENERATED_HEADER "${GENERATED_PATH}/${SCHEMA_NAME}_generated.h")
    add_custom_command(
        OUTPUT "${GENERATED_HEADER}"
        COMMAND ${FLATBUFFERS_FLATC_EXECUTABLE} -c -o "${GENERATED_PATH}" "${SCHEMA_FILE}"
        DEPENDS "${SCHEMA_FILE}"
        COMMENT "Generating Flatbuffers header for ${SCHEMA_FILE}"
    )
    set(FB_GENERATED_HEADERS ${FB_GENERATED_HEADERS} ${GENERATED_HEADER} PARENT_SCOPE)
endfunction()

set(SCHEMA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/schema")
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GEN_DIR})

build_flatbuffers("${SCHEMA_DIR}/position.fbs" "${GEN_DIR}")

# Producer App
add_executable(producer_app producer_app/main.cc ${FB_GENERATED_HEADERS})
target_include_directories(producer_app PRIVATE ${GEN_DIR})
target_link_libraries(producer_app PRIVATE 
    zenohcxx::zenohc
    flatbuffers::flatbuffers
)

# Renderer App
add_executable(renderer_app renderer_app/main.cc ${FB_GENERATED_HEADERS})
target_include_directories(renderer_app PRIVATE ${GEN_DIR} ${GLFW3_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS})
if (GLEW_FOUND)
  target_include_directories(renderer_app PRIVATE ${GLEW_INCLUDE_DIRS})
endif()

target_link_libraries(renderer_app PRIVATE 
    gz-rendering::gz-rendering
    zenohcxx::zenohc
    flatbuffers::flatbuffers
    glfw
    ${OPENGL_LIBRARIES}
)

if (GLEW_FOUND)
  target_link_libraries(renderer_app PRIVATE ${GLEW_LIBRARIES})
endif()
