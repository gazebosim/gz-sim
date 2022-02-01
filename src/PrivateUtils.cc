
#include "PrivateUtils.hh"

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Physics.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
//////////////////////////////////////////////////
void createPhysicsEntities(const sdf::World *_world,
    const Entity &_worldEntity, EntityComponentManager &_ecm)
{
  for (uint64_t index = 0; index < _world->PhysicsCount(); ++index)
  {
    const sdf::Physics *physics = _world->PhysicsByIndex(index);
    if (!physics)
      continue;

    Entity physicsEntity = _ecm.CreateEntity();
    _ecm.CreateComponent(physicsEntity, components::Physics(*physics));
    _ecm.CreateComponent(physicsEntity, components::Name(physics->Name()));

    // Set parent-child relationship
    _ecm.SetParentEntity(physicsEntity, _worldEntity);
    _ecm.CreateComponent(physicsEntity, components::ParentEntity(_worldEntity));

    // Set the world's active physics entity.
    if (physics->IsDefault() || !_ecm.EntityHasComponentType(_worldEntity,
          components::ActivePhysicsEntity::typeId))
    {
      _ecm.CreateComponent(_worldEntity,
          components::ActivePhysicsEntity(physicsEntity));
    }

    // Populate physics options that aren't accessible outside the Element()
    // See https://github.com/osrf/sdformat/issues/508
    if (physics->Element() && physics->Element()->HasElement("dart"))
    {
      auto dartElem = physics->Element()->GetElement("dart");

      if (dartElem->HasElement("collision_detector"))
      {
        auto collisionDetector =
            dartElem->Get<std::string>("collision_detector");

        _ecm.CreateComponent(physicsEntity,
            components::PhysicsCollisionDetector(collisionDetector));
      }
      if (dartElem->HasElement("solver") &&
          dartElem->GetElement("solver")->HasElement("solver_type"))
      {
        auto solver =
            dartElem->GetElement("solver")->Get<std::string>("solver_type");

        _ecm.CreateComponent(physicsEntity, components::PhysicsSolver(solver));
      }
    }
  }
}
}
}
}
