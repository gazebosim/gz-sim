#include "MujocoComponents.hh"

using namespace gz;
using namespace sim;
using namespace systems;
using namespace mujoco_physics;

GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoBodyId", gz::sim::systems::mujoco_physics::MujocoBodyId)
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoGeomId", gz::sim::systems::mujoco_physics::MujocoGeomId)
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoJointId", gz::sim::systems::mujoco_physics::MujocoJointId)
