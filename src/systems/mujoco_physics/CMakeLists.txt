find_package(mujoco REQUIRED)

gz_add_system(mujoco-physics
  SOURCES
    MujocoPhysics.cc
  PUBLIC_LINK_LIBS
    mujoco::mujoco
)
