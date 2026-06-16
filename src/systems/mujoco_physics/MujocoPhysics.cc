#include "MujocoPhysics.hh"
#include "MujocoComponents.hh"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>

#include <mujoco/mujoco.h>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace mujoco_physics;

class gz::sim::systems::mujoco_physics::MujocoPhysicsPrivate
{
  public: mjSpec* spec = nullptr;
  public: mjModel* model = nullptr;
  public: mjData* data = nullptr;

  public: ~MujocoPhysicsPrivate()
  {
    if (this->data)
      mj_deleteData(this->data);
    if (this->model)
      mj_deleteModel(this->model);
    if (this->spec)
      mj_deleteSpec(this->spec);
  }
};

MujocoPhysics::MujocoPhysics()
    : dataPtr(std::make_unique<MujocoPhysicsPrivate>())
{
}

MujocoPhysics::~MujocoPhysics() = default;

void MujocoPhysics::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &_eventMgr)
{
  // Initialize mjSpec
  this->dataPtr->spec = mj_makeSpec();
  
  // Extract gravity from ECM
  auto gravityComp = _ecm.Component<components::Gravity>(_entity);
  if (gravityComp)
  {
    auto gravity = gravityComp->Data();
    this->dataPtr->spec->option.gravity[0] = gravity.X();
    this->dataPtr->spec->option.gravity[1] = gravity.Y();
    this->dataPtr->spec->option.gravity[2] = gravity.Z();
  }

  // Compile the model initially
  this->dataPtr->model = mj_compile(this->dataPtr->spec, nullptr);
  if (this->dataPtr->model)
  {
    this->dataPtr->data = mj_makeData(this->dataPtr->model);
  }
  else
  {
    gzerr << "Failed to compile MuJoCo model in Configure\n";
  }
}

void MujocoPhysics::Update(const UpdateInfo &_info,
                           EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->dataPtr->model && this->dataPtr->data)
  {
    // Step the simulation
    mj_step(this->dataPtr->model, this->dataPtr->data);

    // Synchronize the ECM components::Pose of all links
    // NOTE: The implementation plan suggested doing this in PostUpdate, but 
    // PostUpdate provides a const EntityComponentManager, so mutating components
    // there is impossible and thread-unsafe. It must be done in Update.
    _ecm.Each<components::Link, MujocoBodyId, components::Pose>(
      [&](const Entity &,
          const components::Link *,
          const MujocoBodyId *_bodyIdComp,
          components::Pose *_poseComp) -> bool
      {
        int mjBodyId = _bodyIdComp->Data();
        if (mjBodyId > 0 && mjBodyId < this->dataPtr->model->nbody)
        {
          // mjData->xpos is [nbody x 3], mjData->xquat is [nbody x 4] (w,x,y,z)
          mjtNum* pos = this->dataPtr->data->xpos + 3 * mjBodyId;
          mjtNum* quat = this->dataPtr->data->xquat + 4 * mjBodyId;

          _poseComp->Data() = math::Pose3d(
            pos[0], pos[1], pos[2],
            quat[0], quat[1], quat[2], quat[3] // math::Quaternion(w, x, y, z)
          );
        }
        return true;
      });
  }
}

void MujocoPhysics::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
  // Left intentionally blank. The ECM synchronization logic has been moved to 
  // Update() to respect the constness of the ECM here.
}

GZ_ADD_PLUGIN(MujocoPhysics,
              System,
              MujocoPhysics::ISystemConfigure,
              MujocoPhysics::ISystemUpdate,
              MujocoPhysics::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(MujocoPhysics, "gz::sim::systems::MujocoPhysics")
