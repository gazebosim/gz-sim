#ifndef GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOPHYSICS_HH_
#define GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOPHYSICS_HH_

#include <unordered_set>
#include <memory>

#include <gz/sim/System.hh>

#include <mujoco/mujoco.h>

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
  class MujocoPhysics
      : public System,
        public ISystemConfigure,
        public ISystemUpdate
  {
    public: MujocoPhysics();

    public: ~MujocoPhysics() override;

    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override;

    private: void CreatePhysicsEntities(EntityComponentManager &_ecm);
    private: void UpdatePhysics(EntityComponentManager &_ecm);
    private: void Step(const UpdateInfo &_info);
    private: void UpdateSim(const UpdateInfo &_info, EntityComponentManager &_ecm);

    private: mjSpec* spec{nullptr};
    private: mjModel* model{nullptr};
    private: mjData* data{nullptr};

    private: std::unordered_set<Entity> worldPoseCmdsToRemove;
  };
}
}
}
}
}

#endif
