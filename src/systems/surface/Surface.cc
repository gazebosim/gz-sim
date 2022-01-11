/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <string>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/World.hh"

#include "Surface.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::SurfacePrivate
{
  /// \brief The link entity
  public: ignition::gazebo::Link link;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Vessel length [m].
  public: double vehicleLength = 4.9;

  /// \brief Vessel width [m].
  public: double vehicleWidth = 2.4;

  /// \brief Demi-hull radius [m].
  public: double hullRadius = 0.213;

  /// \brief Length discretization, i.e., "N"
  public: int numSamples = 2;

  /// \brief Fluid height [m].
  public: double fluidLevel = 0;

  /// \brief Fluid density [kg/m^3].
  public: double fluidDensity = 997.7735;

  /// \brief The world's gravity [m/s^2].
  public: ignition::math::Vector3d gravity;
};


//////////////////////////////////////////////////
Surface::Surface()
  : dataPtr(std::make_unique<SurfacePrivate>())
{
}

//////////////////////////////////////////////////
void Surface::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "No <link_name> specified" << std::endl;
    return;
  }

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    ignerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  // Required parameters.
  if (!_sdf->HasElement("vehicle_length"))
  {
    ignerr << "No <vehicle_length> specified" << std::endl;
    return;
  }
  this->dataPtr->vehicleLength = _sdf->Get<double>("vehicle_length");

  if (!_sdf->HasElement("vehicle_width"))
  {
    ignerr << "No <vehicle_width> specified" << std::endl;
    return;
  }
  this->dataPtr->vehicleWidth = _sdf->Get<double>("vehicle_width");

  if (!_sdf->HasElement("hull_radius"))
  {
    ignerr << "No <hull_radius> specified" << std::endl;
    return;
  }
  this->dataPtr->hullRadius = _sdf->Get<double>("hull_radius");

  // Optional parameters.
  if (_sdf->HasElement("num_samples"))
  {
    this->dataPtr->numSamples = _sdf->Get<int>("num_samples");
  }

  if (_sdf->HasElement("fluid_level"))
  {
    this->dataPtr->fluidLevel = _sdf->Get<double>("fluid_level");
  }

  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Get the gravity from the world.
  auto worldEntity = gazebo::worldEntity(_ecm);
  auto world = gazebo::World(worldEntity);
  auto gravityOpt = world.Gravity(_ecm);
  if (!gravityOpt)
  {
    ignerr << "Unable to get the gravity from the world" << std::endl;
    return;
  }
  this->dataPtr->gravity = *gravityOpt;

  // Create necessary components if not present.
  enableComponent<components::Inertial>(_ecm, this->dataPtr->link.Entity());
  enableComponent<components::WorldPose>(_ecm, this->dataPtr->link.Entity());

  igndbg << "Surface plugin successfully configured with the following "
         << "parameters:" << std::endl;
  igndbg << "  <link_name>: " << linkName << std::endl;
  igndbg << "  <vehicle_length>: " << this->dataPtr->vehicleLength << std::endl;
  igndbg << "  <vehicle_width>: " << this->dataPtr->vehicleWidth << std::endl;
  igndbg << "  <hull_radius>: " << this->dataPtr->hullRadius << std::endl;
  igndbg << "  <num_samples>: " << this->dataPtr->numSamples << std::endl;
  igndbg << "  <fluid_level>: " << this->dataPtr->fluidLevel << std::endl;
  igndbg << "  <fluid_density>: " << this->dataPtr->fluidDensity << std::endl;
}

//////////////////////////////////////////////////
void Surface::PreUpdate(const ignition::gazebo::UpdateInfo &/*_info*/,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Surface::PreUpdate");

  // Vehicle frame transform
  const auto kPose = this->dataPtr->link.WorldPose(_ecm);
  // std::optional<ignition::math::Pose3d> kPose;
  // *kPose = {-532, 162, -0.008054, 0, 0, 1};
  if (!kPose)
  {
    ignerr << "Unable to get world pose from link ["
           << this->dataPtr->link.Entity() << "]" << std::endl;
    return;
  }
  const ignition::math::Vector3d kEuler = (*kPose).Rot().Euler();
  ignition::math::Quaternion vq(kEuler.X(), kEuler.Y(), kEuler.Z());

  // Loop over boat grid points
  // Grid point location in boat frame - might be able to precalculate these?
  ignition::math::Vector3d bpnt(0, 0, 0);
  // Grid point location in world frame
  ignition::math::Vector3d bpntW(0, 0, 0);
  // For each hull
  // Debug output:
  // igndbg << "===" << std::endl;
  for (int i = 0; i < 2; ++i)
  {
    // Grid point in boat frame
    bpnt.Set(bpnt.X(), (i * 2.0 - 1.0) * this->dataPtr->vehicleWidth / 2.0,
      bpnt.Z());

    // For each length segment
    for (int j = 1; j <= this->dataPtr->numSamples; ++j)
    {
      bpnt.Set(((j - 0.5) / (static_cast<float>(this->dataPtr->numSamples)) -
        0.5) * this->dataPtr->vehicleLength, bpnt.Y(), bpnt.Z());

      // Transform from vessel to fluid/world frame.
      bpntW = vq * bpnt;

      // Vertical location of boat grid point in world frame.
      const float kDdz = (*kPose).Pos().Z() + bpntW.Z();

      // Find vertical displacement of wave field
      // World location of grid point
      ignition::math::Vector3d X;
      X.X() = (*kPose).Pos().X() + bpntW.X();
      X.Y() = (*kPose).Pos().Y() + bpntW.Y();

      // Compute the depth at the grid point.
      // ToDo: Add wave height here.
      double depth = 0;

      // Vertical wave displacement.
      double dz = depth + X.Z();

      // Total z location of boat grid point relative to fluid surface
      double deltaZ = (this->dataPtr->fluidLevel + dz) - kDdz;
      // enforce only upward buoy force
      deltaZ = std::max(deltaZ, 0.0);
      deltaZ = std::min(deltaZ, this->dataPtr->hullRadius);

      // Buoyancy force at grid point
      const float kBuoyForce =
        this->CircleSegment(this->dataPtr->hullRadius, deltaZ) *
          this->dataPtr->vehicleLength /
          (static_cast<float>(this->dataPtr->numSamples)) *
          -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity;

      // Apply force at grid point
      // Position is in the link frame and force is in world frame.
      this->dataPtr->link.AddWorldForce(_ecm,
        ignition::math::Vector3d(0, 0, kBuoyForce),
        bpnt);

      // Debug output:
      // igndbg << bpnt.X() << "," << bpnt.Y() << "," << bpnt.Z() << std::endl;
      // igndbg << "-" << std::endl;
      // igndbg << bpntW.X() << "," << bpntW.Y() << "," << bpntW.Z() << std::endl;
      // igndbg << "X: " << X << std::endl;
      // igndbg << "depth: " << depth << std::endl;
      // igndbg << "dz: " << dz << std::endl;
      // igndbg << "kDdz: " << kDdz << std::endl;
      // igndbg << "deltaZ: " << deltaZ << std::endl;
      // igndbg << "hull radius: " << this->dataPtr->hullRadius << std::endl;
      // igndbg << "vehicle length: " << this->dataPtr->vehicleLength << std::endl;
      // igndbg << "num samples: " << this->dataPtr->numSamples << std::endl;
      // igndbg << "gravity z: " << -this->dataPtr->gravity.Z() << std::endl;
      // igndbg << "fluid density: " << this->dataPtr->fluidDensity << std::endl;
      // igndbg << "Force: " << kBuoyForce << std::endl << std::endl;
    }
  }
}

//////////////////////////////////////////////////
double Surface::CircleSegment(double _r, double _h) const
{
  return _r * _r * acos((_r -_h) / _r ) -
    (_r - _h) * sqrt(2 * _r * _h - _h * _h);
}

IGNITION_ADD_PLUGIN(Surface,
                    ignition::gazebo::System,
                    Surface::ISystemConfigure,
                    Surface::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Surface,
                          "ignition::gazebo::systems::Surface")
