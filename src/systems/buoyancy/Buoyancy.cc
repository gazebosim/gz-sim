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

#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ExternalWorldWrenchCmd.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "Buoyancy.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::BuoyancyPrivate
{
  /// \brief A class for storing the volume properties of a link.
  public: class VolumeProperties
  {
    /// \brief Center of volume in the link frame.
    public: ignition::math::Vector3d centerOfVolume;

    /// \brief Volume of this link.
    public: double volume{0};
  };

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Pointer to the gravity component.
  public: const components::Gravity *gravity{nullptr};

  /// \brief The density of the fluid in which the object is submerged in
  /// kg/m^3. Defaults to 1000, the fluid density of water.
  public: double fluidDensity{1000};

  /// \brief Map of <link ID, point> pairs mapping link IDs to the CoV (center
  /// of volume) and volume of the link.
  public: std::map<int, VolumeProperties> volPropsMap;
};

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
  // Store this model.
  this->dataPtr->model = Model(_entity);

  // Make sure this plugin is attached to a model.
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "Buoyancy plugin should be attached to a model "
      << "entity. Failed to initialize." << std::endl;
    return;
  }

  // TODO(anyone) If systems are assumed to only have one world, we should
  // capture the world Entity in a Configure call
  Entity world = _ecm.EntityByComponents(components::World());

  if (world == kNullEntity)
  {
    ignerr << "Missing world entity.\n";
    return;
  }

  // Get the gravity (defined in world frame)
  this->dataPtr->gravity = _ecm.Component<components::Gravity>(world);
  if (!this->dataPtr->gravity)
  {
    ignerr << "World is missing gravity." << std::endl;
    return;
  }

  // Store the fluid_density, if specified.
  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Capture the volume of the model by iterating over all the collision
  // elements and storing each geometry's volume.
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::Link());
  for (const Entity &link : links)
  {
    std::vector<Entity> collisions = _ecm.ChildrenByComponents(
        link, components::Collision());

    double volumeSum = 0;
    ignition::math::Vector3d weightedPosSum = ignition::math::Vector3d::Zero;

    for (const Entity &collision : collisions)
    {
      double volume = 0;
      const components::CollisionElement *coll =
        _ecm.Component<components::CollisionElement>(collision);

      if (!coll)
      {
        std::cerr << "Invalid collision??\n";
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
          ignwarn << "Plane shapes are not supported by the Buoyancy plugin.\n";
          break;
        case sdf::GeometryType::MESH:
          {
            std::string file = coll->Data().Geom()->MeshShape()->FilePath();
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
      math::Pose3d pose = worldPose(collision, _ecm);
      weightedPosSum += volume * pose.Pos();
    }

    math::Pose3d pose = worldPose(link, _ecm);
    this->dataPtr->volPropsMap[link].centerOfVolume =
      weightedPosSum / volumeSum - pose.Pos();
    this->dataPtr->volPropsMap[link].volume = volumeSum;
  }
}

//////////////////////////////////////////////////
void Buoyancy::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Buoyancy::PreUpdate");

  // Only update if not paused, we were able to get the gravity vector, and
  // fill the volume property map.
  if (_info.paused || !this->dataPtr->gravity ||
      this->dataPtr->volPropsMap.empty())
  {
    return;
  }

  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::Link());
  for (const Entity &link : links)
  {
    // The volume properties of the link
    BuoyancyPrivate::VolumeProperties volumeProperties =
      this->dataPtr->volPropsMap[link];

    // The link's inertia
    const components::Inertial *inertial =
      _ecm.Component<components::Inertial>(link);

    // World pose of the link.
    math::Pose3d linkWorldPose = worldPose(link, _ecm);

    // By Archimedes' principle,
    // buoyancy = -(mass*gravity)*fluid_density/object_density
    // object_density = mass/volume, so the mass term cancels.
    math::Vector3d buoyancy = -this->dataPtr->fluidDensity *
      volumeProperties.volume *
      this->dataPtr->gravity->Data();

    // Get the offset of the center of volume from the interial frame.
    math::Vector3d offset = volumeProperties.centerOfVolume -
      inertial->Data().Pose().Pos();
    // Convert the offset to the world frame
    math::Vector3d offsetWorld = linkWorldPose.Rot().RotateVector(offset);
    // Compute the torque that should be applied due to buoyancy and
    // the volume offset.
    math::Vector3d torque = offsetWorld.Cross(buoyancy);

    // Apply the wrench to the link. This wrench is applied in the
    // Physics System.
    msgs::Wrench wrench;
    msgs::Set(wrench.mutable_force(), buoyancy);
    msgs::Set(wrench.mutable_torque(), torque);
    components::ExternalWorldWrenchCmd newWrenchComp(wrench);

    auto currWrenchComp =
      _ecm.Component<components::ExternalWorldWrenchCmd>(link);
    if (currWrenchComp)
    {
      // \todo(nkoenig) This overwrites the current wrench component. When
      // we added propellers to a vehicle, then we will need to add the
      // force applied by the propellers to the force applied by buoyancy.
      *currWrenchComp = newWrenchComp;
    }
    else
    {
      _ecm.CreateComponent(link, newWrenchComp);
    }
  }
}

IGNITION_ADD_PLUGIN(Buoyancy,
                    ignition::gazebo::System,
                    Buoyancy::ISystemConfigure,
                    Buoyancy::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Buoyancy,
                          "ignition::gazebo::systems::Buoyancy")
