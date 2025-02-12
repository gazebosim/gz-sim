# find ros
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(self_msg REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
# include_directories(${self_msg_INCLUDE_DIRS})

gz_add_system(wind-effects-pub
  SOURCES
    WindEffectsPub.cc
  PUBLIC_LINK_LIBS
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    # Include gz-sensors for noise models
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    ${rclcpp_LIBRARIES}
)
