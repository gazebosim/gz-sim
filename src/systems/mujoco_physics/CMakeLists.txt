find_package(mujoco REQUIRED)

gz_add_system(mujoco-physics
  SOURCES
    MujocoComponents.cc
    MujocoPhysics.cc
  PUBLIC_LINK_LIBS
    mujoco::mujoco
)
