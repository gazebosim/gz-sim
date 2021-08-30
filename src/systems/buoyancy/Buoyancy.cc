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

#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "ignition/gazebo/components/CenterOfVolume.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Volume.hh"
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
  public: enum BuoyancyType
  {
    /// \brief Applies same buoyancy to whole world.
    UNIFORM_BUOYANCY,
    /// \brief Uses z-axis to determine buoyancy of the world
    /// This is useful for worlds where we want to simulate the ocean interface.
    /// Or for instance if we want to simulate different levels of buoyancies
    /// at different depths.
    GRADED_BUOYANCY
  };
  public: BuoyancyType buoyancyType{BuoyancyType::UNIFORM_BUOYANCY};
  /// \brief Get the fluid density based on a pose.
  /// \param[in] _pose The pose to use when computing the fluid density. The
  /// pose frame is left undefined because this function currently returns
  /// a constant value, see the todo in the function implementation.
  /// \return The fluid density at the givein pose.
  public: double UniformFluidDensity(const math::Pose3d &_pose) const;

  /// \brief Get the resultant buoyant force on a shape.
  /// \param[in] _pose The pose of the shape.
  /// \param[in] _shape The collision mesh of a shape. Currently must
  /// be one of box, cylinder or sphere.
  /// \return a pair containing {force, center_of_volume} to be applied on the
  /// link.
  public:
  template<typename T>
  void GradedFluidDensity(
    const math::Pose3d &_pose, const T &_shape, const math::Vector3d _gravity);

  /// \brief Model interface
  public: Entity world{kNullEntity};

  /// \brief The density of the fluid in which the object is submerged in
  /// kg/m^3. Defaults to 1000, the fluid density of water.
  public: double fluidDensity{1000};

  /// \brief When using GradedBuoyancy, we provide a different buoyancy for
  /// each layer. The key on this map is height in meters and the value is fluid
  /// density. I.E all the fluid between $key$m and $next_key$m has the density
  /// $value$kg/m^3. Everything below the first key is considered as having
  /// fluidDensity.
  public: std::map<double, double> layers;

  public: struct BuoyancyActionPoint
  {
    math::Vector3d force;
    math::Vector3d point;
    math::Pose3d pose;
  };

  public: std::vector<BuoyancyActionPoint> buoyancyForces;

  public: std::pair<math::Vector3d, math::Vector3d> resolveForces(
    const math::Pose3d &_pose);
};

//////////////////////////////////////////////////
double BuoyancyPrivate::UniformFluidDensity(const math::Pose3d &/*_pose*/) const
{
  return this->fluidDensity;
}

//////////////////////////////////////////////////
template<typename T>
void BuoyancyPrivate::GradedFluidDensity(
  const math::Pose3d &_pose, const T &_shape, const math::Vector3d _gravity)
{
  auto prevLayerFluidDensity = this->fluidDensity;
  auto volsum = 0.0;
  auto centerOfBuoyancy = math::Vector3d{0, 0, 0};

  for(auto [height, currFluidDensity] : layers)
  {
    // Transform plane and slice the shape
    math::Planed plane{math::Vector3d{0, 0, 1}, height - _pose.Pos().Z()};
    auto vol = _shape.VolumeBelow(plane);

    // Archimedes principal for this layer
    auto forceMag =  - (vol - volsum) * _gravity * prevLayerFluidDensity;

    // Accumulate layers.
    prevLayerFluidDensity = currFluidDensity;

    // Calculate point from which force is applied
    auto cov = _shape.CenterOfVolumeBelow(plane);

    if(!cov.has_value()) continue;

    auto cob = (cov.value() * vol - centerOfBuoyancy * volsum) / (vol - volsum);
    centerOfBuoyancy = cov.value();
    auto buoyancyAction = BuoyancyActionPoint
    {
      forceMag,
      cob,
      _pose
    };
    buoyancyForces.push_back(buoyancyAction);

    volsum = vol;
  }
  // For the rest of the layers.
  auto vol = _shape.Volume();

  // Archimedes principal for this layer
  auto forceMag = - (vol - volsum) * _gravity * prevLayerFluidDensity;

  // No force contributed by this layer.
  if ((vol - volsum) == 0) return;

  // Calculate centre of buoyancy
  auto cov = math::Vector3d{0, 0, 0};
  auto cob = (cov * vol - centerOfBuoyancy * volsum) / (vol - volsum);
  centerOfBuoyancy = cov;
  auto buoyancyAction = BuoyancyActionPoint
  {
    forceMag,
    cob,
    _pose
  };
  buoyancyForces.push_back(buoyancyAction);
}

//////////////////////////////////////////////////
std::pair<math::Vector3d, math::Vector3d> BuoyancyPrivate::resolveForces(
  const math::Pose3d &_pose)
{
  auto force = math::Vector3d{0, 0, 0};
  auto torque = math::Vector3d{0, 0, 0};

  for(auto b : this->buoyancyForces)
  {
    force += b.force;
    math::Pose3d localPoint{b.point, math::Quaterniond{1, 0, 0, 0}};
    auto globalPoint = b.pose * localPoint;
    auto offset = globalPoint.Pos() - _pose.Pos();
    torque += force.Cross(offset);
  }

  return {force, torque};
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
  else if(_sdf->HasElement("graded_buoyancy"))
  {
    this->dataPtr->buoyancyType =
      BuoyancyPrivate::BuoyancyType::GRADED_BUOYANCY;

    auto gradedElement = _sdf->GetFirstElement();
    if(gradedElement == nullptr)
    {
      ignerr << "Unable to get element description" << std::endl;
      return;
    }

    auto argument = gradedElement->GetFirstElement();
    while(argument != nullptr)
    {
      if(argument->GetName() == "default_density")
      {
        argument->GetValue()->Get<double>(this->dataPtr->fluidDensity);
        igndbg << "Default density set to "
          << this->dataPtr->fluidDensity << std::endl;
      }
      if(argument->GetName() == "density_change")
      {
        auto depth = argument->Get<double>("above_depth", 0.0);
        auto density = argument->Get<double>("density", 0.0);
        if(!depth.second)
          ignwarn << "No <above_depth> tag was found as a "
            << "child of <density_change>" << std::endl;
        if(!density.second)
          ignwarn << "No <density> tag was found as a "
            << "child of <density_change>" << std::endl;
        this->dataPtr->layers[depth.first] = density.first;
        igndbg << "Added layer at " << depth.first << ", "
          <<  density.first << std::endl;
      }
      argument = argument->GetNextElement();
    }
  }
}

//////////////////////////////////////////////////
void Buoyancy::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
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

    enableComponent<components::Inertial>(_ecm, _entity);
    enableComponent<components::WorldPose>(_ecm, _entity);

    Link link(_entity);

    std::vector<Entity> collisions = _ecm.ChildrenByComponents(
        _entity, components::Collision());

    double volumeSum = 0;
    math::Vector3d weightedPosSum = math::Vector3d::Zero;

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
      math::Pose3d pose = worldPose(collision, _ecm);
      weightedPosSum += volume * pose.Pos();
    }

    if (volumeSum > 0)
    {
      // Store the center of volume
      math::Pose3d linkWorldPose = link.WorldInertialPose(_ecm).value();
      _ecm.CreateComponent(_entity, components::CenterOfVolume(
            weightedPosSum / volumeSum - linkWorldPose.Pos()));

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

      Link link(_entity);

      math::Vector3d buoyancy;
      // By Archimedes' principle,
      // buoyancy = -(mass*gravity)*fluid_density/object_density
      // object_density = mass/volume, so the mass term cancels.
      if(this->dataPtr->buoyancyType
        == BuoyancyPrivate::BuoyancyType::UNIFORM_BUOYANCY)
      {
        buoyancy =
        -this->dataPtr->UniformFluidDensity(linkWorldPose) *
        _volume->Data() * gravity->Data();

        // Convert the center of volume to the world frame
        math::Vector3d offsetWorld = linkWorldPose.Rot().RotateVector(
            _centerOfVolume->Data());
        // Compute the torque that should be applied due to buoyancy and
        // the center of volume.
        math::Vector3d torque = offsetWorld.Cross(buoyancy);

        // Apply the wrench to the link. This wrench is applied in the
        // Physics System.
        link.AddWorldWrench(_ecm, buoyancy, torque);
      }
      else if(this->dataPtr->buoyancyType
        == BuoyancyPrivate::BuoyancyType::GRADED_BUOYANCY)
      {
        std::vector<Entity> collisions = _ecm.ChildrenByComponents(
          _entity, components::Collision());
        this->dataPtr->buoyancyForces.clear();

        for(auto e : collisions)
        {
          const components::CollisionElement *coll =
            _ecm.Component<components::CollisionElement>(e);

          auto pose = worldPose(e, _ecm);

          if (!coll)
          {
            ignerr << "Invalid collision pointer. This shouldn't happen\n";
            continue;
          }

          switch (coll->Data().Geom()->Type())
          {
            case sdf::GeometryType::BOX:
              this->dataPtr->GradedFluidDensity<math::Boxd>(
                pose,
                coll->Data().Geom()->SphereShape()->Shape(),
                gravity->Data());
              break;
            case sdf::GeometryType::SPHERE:
              this->dataPtr->GradedFluidDensity<math::Sphered>(
                pose,
                coll->Data().Geom()->SphereShape()->Shape(),
                gravity->Data());
              break;
            default:
              ignerr << "Only <box> and <sphere> collisions are supported by "
                << "the graded buoyancy option." << std::endl;
              break;
          }
        }
      }
      auto [force, torque]= this->dataPtr->resolveForces(
        link.WorldInertialPose(_ecm).value());
      // Apply the wrench to the link. This wrench is applied in the
      // Physics System.
      link.AddWorldWrench(_ecm, force, torque);
      return true;
  });
}

IGNITION_ADD_PLUGIN(Buoyancy,
                    ignition::gazebo::System,
                    Buoyancy::ISystemConfigure,
                    Buoyancy::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Buoyancy,
                          "ignition::gazebo::systems::Buoyancy")
