#ifndef GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOPHYSICS_HH_
#define GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOPHYSICS_HH_

#include <memory>

#include <gz/sim/System.hh>

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
  class MujocoPhysicsPrivate;

  class MujocoPhysics
      : public System,
        public ISystemConfigure,
        public ISystemUpdate,
        public ISystemPostUpdate
  {
    public: MujocoPhysics();

    public: ~MujocoPhysics() override;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override;

    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;

    private: std::unique_ptr<MujocoPhysicsPrivate> dataPtr;
  };
}
}
}
}
}

#endif
