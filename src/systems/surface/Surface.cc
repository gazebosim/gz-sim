/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <unordered_set>
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
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Volume.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "Surface.hh"

#define GRAVITY 9.815

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
  public: double paramBoatLength = 4.9;

  /// \brief Vessel width [m].
  public: double paramBoatWidth = 2.4;

  /// \brief Demi-hull radius [m].
  public: double paramHullRadius = 0.213;

  /// \brief Length discretization, i.e., "N"
  public: int paramLengthN = 2;

  /// \brief Water height [m].
  public: double waterLevel = 0;

  /// \brief Water density [kg/m^3].
  public: double waterDensity = 997.7735;
};


//////////////////////////////////////////////////
Surface::Surface()
  : dataPtr(std::make_unique<SurfacePrivate>())
{
  ignerr << "Surface plugin loaded!" << std::endl;
}

//////////////////////////////////////////////////
void Surface::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  std::string linkName = "base_link";

  this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    ignerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  ignmsg << "Surface plugin configured!" << std::endl;

  // // Create necessary components if not present.
  // enableComponent<components::WorldPose>(_ecm, this->dataPtr->linkEntity);
}

//////////////////////////////////////////////////
void Surface::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Surface::PreUpdate");

  // Vehicle frame transform
  // tf2::Quaternion vq = tf2::Quaternion();
  // tf2::Matrix3x3 m;
  // m.setEulerYPR(kEuler.Z(), kEuler.Y(), kEuler.X());
  // m.getRotation(vq);
  // tf2::Transform xformV = tf2::Transform(vq);
  const auto kPose = this->dataPtr->link.WorldPose(_ecm);
  const ignition::math::Vector3d kEuler = (*kPose).Rot().Euler();
  ignition::math::Quaternion vq(kEuler.X(), kEuler.Y(), kEuler.Z());
  ignerr << "Angles: " << kEuler.X() << "," << kEuler.Y() << "," << kEuler.Z() << std::endl;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  // #if GAZEBO_MAJOR_VERSION >= 8
  //   const ignition::math::Pose3d kPose = this->link->WorldPose();
  // #else
  //   const ignition::math::Pose3d kPose = this->link->GetWorldPose().Ign();
  // #endif
  // const ignition::math::Vector3d kEuler = kPose.Rot().Euler();

  //ignition::gazebo::Link link(this->dataPtr->linkEntity);
  //auto pose = worldPose(this->dataPtr->link, _ecm);


  // Loop over boat grid points
  // Grid point location in boat frame - might be able to precalculate these?
  ignition::math::Vector3d bpnt(0, 0, 0);
  // Grid point location in world frame
  ignition::math::Vector3d bpntW(0, 0, 0);
  // For each hull
  ignerr << "===" << std::endl;
  for (int ii = 0; ii < 2; ii++)
  {
    // Grid point in boat frame
    bpnt.Set(bpnt.X(), (ii*2.0-1.0)*this->dataPtr->paramBoatWidth/2.0, bpnt.Z());

    // For each length segment
    for (int jj = 1; jj <= this->dataPtr->paramLengthN; jj++)
    {
      bpnt.Set(((jj - 0.5) / (static_cast<float>(this->dataPtr->paramLengthN)) -
        0.5) * this->dataPtr->paramBoatLength, bpnt.Y(), bpnt.Z());

      // Transform from vessel to water/world frame
      // ToDo(caguero): fix
      // bpntW = xformV * bpnt;
      bpntW = vq * bpnt;

      ignerr << bpnt.X() << ", " << bpnt.Y() << ", " << bpnt.Z() << std::endl;
      ignerr << "-" << std::endl;
      ignerr << bpntW.X() << ", " << bpntW.Y() << ", " << bpntW.Z() << std::endl;

      // Vertical location of boat grid point in world frame
      const float kDdz = (*kPose).Pos().Z() + bpntW.Z();
      // ROS_DEBUG_STREAM("Z, pose: " << (*kPose).Pos().Z() << ", bpnt: "
      //   << bpntW.Z() << ", dd: " << kDdz);

      // Find vertical displacement of wave field
      // World location of grid point
      ignition::math::Vector3d X;
      X.X() = (*kPose).Pos().X() + bpntW.X();
      X.Y() = (*kPose).Pos().Y() + bpntW.Y();

      // Compute the depth at the grid point.
      // double simTime = kTimeNow.Double();
      // double depth = WavefieldSampler::ComputeDepthDirectly(
      //  *waveParams, X, simTime);
      double depth = 0.0;
      // if (waveParams)
      // {
      //   depth = WavefieldSampler::ComputeDepthSimply(*waveParams, X, simTime);
      // }

      // Vertical wave displacement.
      double dz = depth + X.Z();
      ignerr << "depth: " << depth << std::endl;
      ignerr << "x.Z(): " << X.Z() << std::endl;

      // Total z location of boat grid point relative to water surface
      double deltaZ = (this->dataPtr->waterLevel + dz) - kDdz;
      ignerr << "waterLevel: " << this->dataPtr->waterLevel << std::endl;
      ignerr << "dz: " << dz << std::endl;
      ignerr << "kDdz: " << kDdz << std::endl;
      ignerr << "deltaZ: " << deltaZ << std::endl;
      deltaZ = std::max(deltaZ, 0.0);  // enforce only upward buoy force
      ignerr << "deltaZ: " << deltaZ << std::endl;
      deltaZ = std::min(deltaZ, this->dataPtr->paramHullRadius);
      ignerr << "deltaZ: " << deltaZ << std::endl;

      // Buoyancy force at grid point
      const float kBuoyForce =
        CircleSegment(this->dataPtr->paramHullRadius, deltaZ) *
        this->dataPtr->paramBoatLength /
        (static_cast<float>(this->dataPtr->paramLengthN)) *
        GRAVITY * this->dataPtr->waterDensity;
      //ROS_DEBUG_STREAM("buoyForce: " << kBuoyForce);

      // Apply force at grid point
      // From web, Appears that position is in the link frame
      // and force is in world frame
      this->dataPtr->link.AddWorldForce(_ecm,
       ignition::math::Vector3d(0, 0, kBuoyForce),
       bpnt);

      ignerr << "Force: " << kBuoyForce << std::endl;
    }
  }
}

//////////////////////////////////////////////////
double Surface::CircleSegment(double R, double h)
{
  return R*R*acos( (R-h)/R ) - (R-h)*sqrt(2*R*h-h*h);
}

IGNITION_ADD_PLUGIN(Surface,
                    ignition::gazebo::System,
                    Surface::ISystemConfigure,
                    Surface::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(Surface,
                          "ignition::gazebo::systems::Surface")
