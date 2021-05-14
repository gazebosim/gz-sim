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
  /// \brief Get the fluid density based on a pose. This function can be
  /// used to adjust the fluid density based on the pose of an object in the
  /// world. This function currently returns a constant value, see the todo
  /// in the function implementation.
  /// \param[in] _pose The pose to use when computing the fluid density. The
  /// pose frame is left undefined because this function currently returns
  /// a constant value, see the todo in the function implementation.
  /// \return The fluid density at the givein pose.
  public: double UniformFluidDensity(const math::Pose3d &_pose) const;

  /// \brief Model interface
  public: Entity world{kNullEntity};

  /// \brief The density of the fluid in which the object is submerged in
  /// kg/m^3. Defaults to 1000, the fluid density of water.
  public: double fluidDensity{1000};
};


math::Vector3d getPointOnPlane(
  math::Plane<double>& plane,
  double x,
  double y)
{
  auto z_val = (plane.Offset() - (plane.Normal().Dot({1,1,0})))/plane.Normal().Z();
  auto coincidentPoint = math::Vector3d{x,y,z_val};

  IGN_ASSERT(coincidentPoint.Dot(plane.Normal()) == plane.Offset(),
    "Point is not coincident with plane");

  return coincidentPoint;
}


double VolumeBelow(sdf::Sphere sphere, math::Pose3d position, math::Plane<double> plane)
{
  auto r = sphere.Radius();
  
  auto coincidentPoint = getPointOnPlane(plane, 1, 1);

  auto vec = coincidentPoint - position.Pos();
  auto h = r-vec.Dot(plane.Normal())/plane.Normal().Length();

  if(h > 0)
  {
    return IGN_PI*h*h*(3*r-h)/3;
  }
  else
  {
    return 4/3*IGN_PI*r*r*r;
  }
}

///////////////////////////////////////////////////
double VolumeBelow(sdf::Cylinder cylinder, math::Pose3d pos, math::Plane<double> plane)
{
  // This function is extremely hairy it needs to account for 3 different cases
  // 1. Horizontal cut parallel to the cylinder's axis going through both flat
  // faces
  // 2. Horizontal cut going through one flat faces
  // 3. Horizontal cut going through zero flat faces
  auto length = cylinder.Length();
  auto radius = cylinder.Radius();

  auto transformedNormal = pos.Rot().RotateVector(plane.Normal());

  auto z_val = (plane.Offset() - (plane.Normal().Dot({1,1,0})))
    /plane.Normal().Z();
  auto coincidentPoint = math::Vector3d{1,1,z_val};

  //Transform plane into local coordinate frame.
  math::Matrix4d pose(pos);
  auto transformedPoint = pose.TransformAffine(coincidentPoint);
  auto offset = transformedNormal.Dot(transformedPoint);
  math::Plane<double> transformedPlane(transformedNormal, offset);

  //Compute intersection point of plane
  auto theta = atan2(transformedNormal.Y(), transformedNormal.X());
  auto x = radius * cos(theta);
  auto y = radius * sin(theta);
  auto point_max = getPointOnPlane(transformedPlane, x, y);
  x = radius * cos(theta + IGN_PI);
  y = radius * sin(theta + IGN_PI);
  auto point_min = getPointOnPlane(transformedPlane, x, y);

  //Get case type
  if(abs(point_max.Z()) > length/2)
  {
    if(abs(point_min.Z()) > length/2)
    {
      // Plane cuts through both flat faces
    }
    else
    {
      // Cuts through one face
    }
  }
  else if(abs(point_min.Z()) > length/2)
  {
    // Cuts through one face
  }
  else
  {
    // Plane Cuts thoguh no faces.
    auto a = abs(point_max.Z()) + length/2;
    auto b = abs(point_min.Z()) + length/2;
    auto avg_height = (a + b)/2;
    return avg_height * IGN_PI * radius * radius;
  }

}

//////////////////////////////////////////////////
double BuoyancyPrivate::UniformFluidDensity(const math::Pose3d & /*_pose*/) const
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

    Link link(_entity);

    std::vector<Entity> collisions = _ecm.ChildrenByComponents(
        _entity, components::Collision());

    double volumeSum = 0;
    ignition::math::Vector3d weightedPosSum =
      ignition::math::Vector3d::Zero;

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
      math::Pose3d linkWorldPose = worldPose(_entity, _ecm);
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
            components::CenterOfVolume,
            components::CollisionElement>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Volume *_volume,
          const components::CenterOfVolume *_centerOfVolume,
          const components::CollisionElement *_collision_element) -> bool
    {
      // World pose of the link.
      math::Pose3d linkWorldPose = worldPose(_entity, _ecm);

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
      }

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
                    ignition::gazebo::System,
                    Buoyancy::ISystemConfigure,
                    Buoyancy::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Buoyancy,
                          "ignition::gazebo::systems::Buoyancy")
