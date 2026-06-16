find_package(mujoco REQUIRED)

gz_add_system(mujoco_physics
  SOURCES
    MujocoComponents.cc
  PUBLIC_LINK_LIBS
    mujoco::mujoco
)
