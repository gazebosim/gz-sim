/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <ignition/msgs/wrench.pb.h>

#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <ignition/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/CenterOfVolume.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Volume.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "Buoyancy.hh"

using namespace gz;
using namespace gz::sim;
using namespace systems;

class gz::sim::systems::BuoyancyPrivate
{
  /// \brief Get the fluid density based on a pose. This function can be
  /// used to adjust the fluid density based on the pose of an object in the
  /// world. This function currently returns a constant value, see the todo
  /// in the function implementation.
  /// \param[in] _pose The pose to use when computing the fluid density. The
  /// pose frame is left undefined because this function currently returns
  /// a constant value, see the todo in the function implementation.
  /// \return The fluid density at the givein pose.
  public: double FluidDensity(const math::Pose3d &_pose) const;

  /// \brief Model interface
  public: Entity world{kNullEntity};

  /// \brief The density of the fluid in which the object is submerged in
  /// kg/m^3. Defaults to 1000, the fluid density of water.
  public: double fluidDensity{1000};
};

//////////////////////////////////////////////////
double BuoyancyPrivate::FluidDensity(const math::Pose3d & /*_pose*/) const
{
  // \todo(nkoenig) Adjust the fluid density based on the provided pose.
  // This could take into acount:
  //   1. Transition from water to air. Currently this function is used for
  //   a whole link, but when transitioning between mediums a link may span
  //   both mediums. Surface tension could also be a factor.
  //   2. Fluid density changes based on depth below the water surface and
  //   height above water surface.
  return this->fluidDensity;
}

//////////////////////////////////////////////////
Buoyancy::Buoyancy()
  : dataPtr(std::make_unique<BuoyancyPrivate>())
{
}

//////////////////////////////////////////////////
void Buoyancy::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // Store the world.
  this->dataPtr->world = _entity;

  // Get the gravity (defined in world frame)
  const components::Gravity *gravity = _ecm.Component<components::Gravity>(
      this->dataPtr->world);
  if (!gravity)
  {
    ignerr << "Unable to get the gravity vector. Make sure this plugin is "
      << "attached to a <world>, not a <model>." << std::endl;
    return;
  }

  if (_sdf->HasElement("uniform_fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("uniform_fluid_density");
  }
}

//////////////////////////////////////////////////
void Buoyancy::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("Buoyancy::PreUpdate");
  const components::Gravity *gravity = _ecm.Component<components::Gravity>(
      this->dataPtr->world);
  if (!gravity)
  {
    ignerr << "Unable to get the gravity vector. Has gravity been defined?"
           << std::endl;
    return;
  }

  // Compute the volume and center of volume for each new link
  _ecm.EachNew<components::Link, components::Inertial>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Inertial *) -> bool
  {
    // Skip if the entity already has a volume and center of volume
    if (_ecm.EntityHasComponentType(_entity,
          components::CenterOfVolume().TypeId()) &&
        _ecm.EntityHasComponentType(_entity,
          components::Volume().TypeId()))
    {
      return true;
    }

    Link link(_entity);

    std::vector<Entity> collisions = _ecm.ChildrenByComponents(
        _entity, components::Collision());

    double volumeSum = 0;
    math::Vector3d weightedPosInLinkSum =
      math::Vector3d::Zero;

    // Compute the volume of the link by iterating over all the collision
    // elements and storing each geometry's volume.
    for (const Entity &collision : collisions)
    {
      double volume = 0;
      const components::CollisionElement *coll =
        _ecm.Component<components::CollisionElement>(collision);

      if (!coll)
      {
        ignerr << "Invalid collision pointer. This shouldn't happen\n";
        continue;
      }

      switch (coll->Data().Geom()->Type())
      {
        case sdf::GeometryType::BOX:
          volume = coll->Data().Geom()->BoxShape()->Shape().Volume();
          break;
        case sdf::GeometryType::SPHERE:
          volume = coll->Data().Geom()->SphereShape()->Shape().Volume();
          break;
        case sdf::GeometryType::CYLINDER:
          volume = coll->Data().Geom()->CylinderShape()->Shape().Volume();
          break;
        case sdf::GeometryType::PLANE:
          // Ignore plane shapes. They have no volume and are not expected
          // to be buoyant.
          break;
        case sdf::GeometryType::MESH:
          {
            std::string file = asFullPath(
                coll->Data().Geom()->MeshShape()->Uri(),
                coll->Data().Geom()->MeshShape()->FilePath());
            if (common::MeshManager::Instance()->IsValidFilename(file))
            {
              const common::Mesh *mesh =
                common::MeshManager::Instance()->Load(file);
              if (mesh)
                volume = mesh->Volume();
              else
                ignerr << "Unable to load mesh[" << file << "]\n";
            }
            else
            {
              ignerr << "Invalid mesh filename[" << file << "]\n";
            }
            break;
          }
        default:
          ignerr << "Unsupported collision geometry["
            << static_cast<int>(coll->Data().Geom()->Type()) << "]\n";
          break;
      }

      volumeSum += volume;
      auto poseInLink = _ecm.Component<components::Pose>(collision)->Data();
      weightedPosInLinkSum += volume * poseInLink.Pos();
    }

    if (volumeSum > 0)
    {
      // Store the center of volume expressed in the link frame
      _ecm.CreateComponent(_entity, components::CenterOfVolume(
            weightedPosInLinkSum / volumeSum));

      // Store the volume
      _ecm.CreateComponent(_entity, components::Volume(volumeSum));
    }

    return true;
  });

  // Only update if not paused.
  if (_info.paused)
    return;

  _ecm.Each<components::Link,
            components::Volume,
            components::CenterOfVolume>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Volume *_volume,
          const components::CenterOfVolume *_centerOfVolume) -> bool
    {
      // World pose of the link.
      math::Pose3d linkWorldPose = worldPose(_entity, _ecm);

      // By Archimedes' principle,
      // buoyancy = -(mass*gravity)*fluid_density/object_density
      // object_density = mass/volume, so the mass term cancels.
      math::Vector3d buoyancy =
        -this->dataPtr->FluidDensity(linkWorldPose) *
        _volume->Data() * gravity->Data();

      // Convert the center of volume to the world frame
      math::Vector3d offsetWorld = linkWorldPose.Rot().RotateVector(
          _centerOfVolume->Data());
      // Compute the torque that should be applied due to buoyancy and
      // the center of volume.
      math::Vector3d torque = offsetWorld.Cross(buoyancy);

      // Apply the wrench to the link. This wrench is applied in the
      // Physics System.
      Link link(_entity);
      link.AddWorldWrench(_ecm, buoyancy, torque);

      return true;
  });
}

IGNITION_ADD_PLUGIN(Buoyancy,
                    System,
                    Buoyancy::ISystemConfigure,
                    Buoyancy::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Buoyancy,
                          "sim::systems::Buoyancy")
