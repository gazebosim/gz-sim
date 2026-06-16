#ifndef GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOCOMPONENTS_HH_
#define GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOCOMPONENTS_HH_

#include <istream>
#include <ostream>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
namespace mujoco_physics
{
  class NoSerializeSerializer
  {
    public: static std::ostream &Serialize(std::ostream &_out, const int &_data)
    {
      (void)_data;
      return _out;
    }
    public: static std::istream &Deserialize(std::istream &_in, int &_data)
    {
      (void)_data;
      return _in;
    }
  };

  using MujocoBodyId = components::Component<int, class MujocoBodyIdTag, NoSerializeSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoBodyId", MujocoBodyId)

  using MujocoGeomId = components::Component<int, class MujocoGeomIdTag, NoSerializeSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoGeomId", MujocoGeomId)

  using MujocoJointId = components::Component<int, class MujocoJointIdTag, NoSerializeSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoJointId", MujocoJointId)

  using MujocoActuatorId = components::Component<int, class MujocoActuatorIdTag, NoSerializeSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MujocoActuatorId", MujocoActuatorId)
}
}
}
}
}

#endif
