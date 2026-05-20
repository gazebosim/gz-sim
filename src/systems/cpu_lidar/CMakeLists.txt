gz_add_system(cpu-lidar
  SOURCES
    CpuLidar.cc
  PUBLIC_LINK_LIBS
    gz-common::gz-common
  PRIVATE_LINK_LIBS
    gz-sensors::cpu_lidar
)
