project(custom_rendering_sensor)

add_library(${PROJECT_NAME} SHARED CustomRenderingSensor.cc)

target_link_libraries(${PROJECT_NAME}
  PUBLIC 
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
    ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}-rendering
  )
