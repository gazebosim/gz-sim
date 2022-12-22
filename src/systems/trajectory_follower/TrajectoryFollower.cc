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

#include <gz/msgs/boolean.pb.h>

#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>
#include <sdf/sdf.hh>

#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/AngularVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "TrajectoryFollower.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::TrajectoryFollowerPrivate
{
  /// \brief Initialize the plugin.
  /// \param[in] _ecm Immutable reference to the EntityComponentManager.
  /// \param[in] _sdf The SDF Element associated with this system plugin.
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Callback to pause/resume the behavior.
  /// \param[in] _paused True when the intention is to pause the trajectory
  /// follower behavior or false to continue the trajectory.
  public: void OnPause(const msgs::Boolean &_paused);

  /// \brief A mutex to protect the paused member.
  public: std::mutex mutex;

  /// \brief Gazebo transport node.
  public: transport::Node node;

  /// \brief Topic name to pause/resume the trajectory.
  public: std::string topic;

  /// \brief The link entity
  public: gz::sim::Link link;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The initial pose of the model relative to the world frame.
  public: gz::math::Pose3<double> modelPose;

  /// \brief True if the model should continue looping though the waypoints.
  public: bool loopForever = false;

  /// \brief Linear force to apply to the model in its X direction.
  public: double forceToApply = 60;

  /// \brief Torque to apply to the model to align it with the next goal.
  public: double torqueToApply = 50;

  /// \brief When the model is at this distance or closer we won't try to move.
  /// Units are in meters.
  public: double rangeTolerance = 0.5;

  /// \brief When the model is at this angle or closer we won't try to rotate.
  /// Units are in degrees.
  public: double bearingTolerance = 2.0;

  /// \brief Use to sample waypoints around a circle.
  public: unsigned int numSamples = 8u;

  /// \brief The next position to reach.
  public: gz::math::Vector3d nextGoal;

  /// \brief Vector containing waypoints as 3D vectors of doubles representing
  /// X Y, where X and Y are local (Gazebo) coordinates.
  public: std::vector<gz::math::Vector2d> localWaypoints;

  /// \brief Initialization flag.
  public: bool initialized{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Whether the trajectory follower behavior should be paused or not.
  public: bool paused = false;

  /// \brief Angular velocity set to zero
  public: bool zeroAngVelSet = false;

  /// \brief Force angular velocity to be zero when bearing is reached
  public: bool forceZeroAngVel = false;
};

//////////////////////////////////////////////////
void TrajectoryFollowerPrivate::Load(const EntityComponentManager &_ecm,
    const sdf::ElementPtr &_sdf)
{
  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "No <link_name> specified" << std::endl;
    return;
  }

  std::string linkName = _sdf->Get<std::string>("link_name");
  this->link = Link(this->model.LinkByName(_ecm, linkName));
  if (!this->link.Valid(_ecm))
  {
    gzerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  this->modelPose = gz::sim::worldPose(this->link.Entity(), _ecm);
  this->modelPose.Pos().Z() = 0;

  // Parse the optional <waypoints> element.
  if (_sdf->HasElement("waypoints"))
  {
    auto waypointsElem = _sdf->GetElement("waypoints");

    // We need at least one waypoint
    if (!waypointsElem->HasElement("waypoint"))
    {
      gzerr << "TrajectoryFollower: Unable to find <waypoints><waypoint> "
            << "element in SDF." << std::endl;
      return;
    }
    auto waypointElem = waypointsElem->GetElement("waypoint");
    while (waypointElem)
    {
      gz::math::Vector2d position =
        waypointElem->Get<gz::math::Vector2d>();

      // Save the position.
      this->localWaypoints.push_back(position);

      // Print some debugging messages
      gzdbg << "Waypoint, Local: X = " << position.X()
             << " Y = " << position.Y() << std::endl;

      waypointElem = waypointElem->GetNextElement("waypoint");
    }
  }
  // If no waypoints present, check for the <circle> element and parse.
  else if (_sdf->HasElement("circle"))
  {
    gzdbg << "Circle element activated" << std::endl;
    auto circleElem = _sdf->GetElement("circle");

    if (!circleElem->HasElement("radius"))
    {
      gzerr << "No <circle><radius> specified" << std::endl;
      return;
    }

    // Parse the required <radius> field.
    double radius = circleElem->Get<double>("radius");

    // Get the current model position in global coordinates. Create
    // local vectors that represent a path along a rough circle.
    gz::math::Vector2d position(this->modelPose.Pos().X(),
                                      this->modelPose.Pos().Y());
    double angle = 0;
    gz::math::Vector2d vec(radius, 0);
    for (unsigned int i = 0u; i < this->numSamples; ++i)
    {
      // Add the local vector to the current position.
      // Store global position as a waypoint.
      this->localWaypoints.push_back(position + vec);
      angle += 2 * GZ_PI / this->numSamples;
      vec.Set(radius * cos(angle), radius * sin(angle));
      gzdbg << "Entered circle waypoint " << position + vec << std::endl;
    }
  }
  // If no waypoints or circle, check for the <line> element and parse.
  else if (_sdf->HasElement("line"))
  {
    auto lineElem = _sdf->GetElement("line");
    // Parse the required <direction> field.
    if (!lineElem->HasElement("direction"))
    {
      gzerr << "No <line><direction> specified" << std::endl;
      return;
    }
    gz::math::Angle direction =
      lineElem->Get<gz::math::Angle>("direction");

    // Parse the required <length> field.
    if (!lineElem->HasElement("length"))
    {
      gzerr << "No <line><length> specified" << std::endl;
      return;
    }
    auto length = lineElem->Get<double>("length");

    // Create a relative vector in the direction of "direction" and of
    // length "length".
    gz::math::Vector3d lineVec(
      length * cos(direction.Radian()),
      length * sin(direction.Radian()), 0);
    gz::math::Vector2d position(this->modelPose.Pos().X(),
                                      this->modelPose.Pos().Y());
    // Add the initial model position and calculated endpoint as waypoints.
    this->localWaypoints.push_back(position);
    gz::math::Vector3d p = this->modelPose.CoordPositionAdd(lineVec);
    gz::math::Vector2d p2D = {p.X(), p.Y()};
    this->localWaypoints.push_back(p2D);
    gzdbg << "Entered line waypoints " << position << ", " << p2D << std::endl;
  }

  // Parse the optional <loop> element.
  if (_sdf->HasElement("loop"))
    this->loopForever = _sdf->Get<bool>("loop");

  // Parse the optional <force> element.
  if (_sdf->HasElement("force"))
    this->forceToApply = _sdf->Get<double>("force");

  // Parse the optional <torque> element.
  if (_sdf->HasElement("torque"))
    this->torqueToApply = _sdf->Get<double>("torque");

  // Parse the optional <range_tolerance> element.
  if (_sdf->HasElement("range_tolerance"))
    this->rangeTolerance = _sdf->Get<double>("range_tolerance");

  // Parse the optional <bearing_tolerance> element.
  if (_sdf->HasElement("bearing_tolerance"))
    this->bearingTolerance = _sdf->Get<double>("bearing_tolerance");

  // Parse the optional <zero_vel_on_bearing_reached> element.
  if (_sdf->HasElement("zero_vel_on_bearing_reached"))
    this->forceZeroAngVel = _sdf->Get<bool>("zero_vel_on_bearing_reached");

  // Parse the optional <topic> element.
  this->topic = "/model/" + this->model.Name(_ecm) +
    "/trajectory_follower/pause";

  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");

  this->topic = transport::TopicUtils::AsValidTopic(this->topic);

  this->node.Subscribe(topic, &TrajectoryFollowerPrivate::OnPause, this);

  gzmsg << "TrajectoryFollower["
      << this->model.Name(_ecm) << "] subscribed "
      << "to pause messages on topic[" << this->topic << "]\n";

  // If we have waypoints to visit, read the first one.
  if (!this->localWaypoints.empty())
  {
    this->nextGoal =
      {this->localWaypoints.front().X(), this->localWaypoints.front().Y(), 0};
  }
}

/////////////////////////////////////////////////
void TrajectoryFollowerPrivate::OnPause(const msgs::Boolean &_paused)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->paused = _paused.data();
}

//////////////////////////////////////////////////
TrajectoryFollower::TrajectoryFollower()
  : dataPtr(std::make_unique<TrajectoryFollowerPrivate>())
{
}

//////////////////////////////////////////////////
void TrajectoryFollower::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void TrajectoryFollower::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("TrajectoryFollower::PreUpdate");

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (_info.paused || this->dataPtr->paused)
      return;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    enableComponent<components::WorldPose>(_ecm, this->dataPtr->link.Entity());
    this->dataPtr->initialized = true;
  }

  // Nothing to do.
  if (this->dataPtr->localWaypoints.empty())
    return;

  this->dataPtr->modelPose = gz::sim::worldPose(
    this->dataPtr->link.Entity(), _ecm);
  this->dataPtr->modelPose.Pos().Z() = 0;

  // Direction vector to the goal from the model.
  gz::math::Vector3d direction =
    this->dataPtr->nextGoal - this->dataPtr->modelPose.Pos();

  // Direction vector in the local frame of the model.
  gz::math::Vector3d directionLocalFrame =
    this->dataPtr->modelPose.Rot().RotateVectorReverse(direction);

  double range = directionLocalFrame.Length();
  gz::math::Angle bearing(
    atan2(directionLocalFrame.Y(), directionLocalFrame.X()));
  bearing.Normalize();

  // Waypoint reached!
  if (range <= this->dataPtr->rangeTolerance)
  {
    // We always keep the last waypoint in the vector to keep the model
    // "alive" in case it moves away from its goal.
    if (this->dataPtr->localWaypoints.size() == 1)
      return;

    if (this->dataPtr->loopForever)
    {
      // Rotate to the left.
      std::rotate(this->dataPtr->localWaypoints.begin(),
                  this->dataPtr->localWaypoints.begin() + 1,
                  this->dataPtr->localWaypoints.end());
    }
    else
    {
      // Remove the first waypoint.
      this->dataPtr->localWaypoints.erase(
        this->dataPtr->localWaypoints.begin());
    }

    this->dataPtr->nextGoal = {
      this->dataPtr->localWaypoints.front().X(),
      this->dataPtr->localWaypoints.front().Y(), 0};

    return;
  }

  // Transform from world to local frame.
  auto comPose = this->dataPtr->link.WorldInertialPose(_ecm);
  if (!comPose.has_value())
  {
    gzerr << "Failed to get CoM pose for link ["
           << this->dataPtr->link.Entity() << "]" << std::endl;
    return;
  }

  // Transform the force and torque to the world frame.
  // Move commands. The vehicle always move forward (X direction).
  gz::math::Vector3d forceWorld;
  if (std::abs(bearing.Degree()) <= this->dataPtr->bearingTolerance)
  {
    forceWorld = (*comPose).Rot().RotateVector(
      gz::math::Vector3d(this->dataPtr->forceToApply, 0, 0));

    // force angular velocity to be zero when bearing is reached
    if (this->dataPtr->forceZeroAngVel && !this->dataPtr->zeroAngVelSet &&
        math::equal (std::abs(bearing.Degree()), 0.0,
        this->dataPtr->bearingTolerance * 0.5))
    {
      this->dataPtr->link.SetAngularVelocity(_ecm, math::Vector3d::Zero);
      this->dataPtr->zeroAngVelSet = true;
    }
  }
  gz::math::Vector3d torqueWorld;
  if (std::abs(bearing.Degree()) > this->dataPtr->bearingTolerance)
  {
    // remove angular velocity component otherwise the physics system will set
    // the zero ang vel command every iteration
    if (this->dataPtr->forceZeroAngVel && this->dataPtr->zeroAngVelSet)
    {
      auto angVelCmdComp = _ecm.Component<components::AngularVelocityCmd>(
          this->dataPtr->link.Entity());
      if (angVelCmdComp)
      {
        _ecm.RemoveComponent<components::AngularVelocityCmd>(
          this->dataPtr->link.Entity());
        this->dataPtr->zeroAngVelSet = false;
      }
    }

    int sign = static_cast<int>(std::abs(bearing.Degree()) / bearing.Degree());
    torqueWorld = (*comPose).Rot().RotateVector(
       gz::math::Vector3d(0, 0, sign * this->dataPtr->torqueToApply));
  }

  // Apply the force and torque at COM.
  this->dataPtr->link.AddWorldWrench(_ecm, forceWorld, torqueWorld);
}

GZ_ADD_PLUGIN(TrajectoryFollower,
                    gz::sim::System,
                    TrajectoryFollower::ISystemConfigure,
                    TrajectoryFollower::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(TrajectoryFollower,
                          "gz::sim::systems::TrajectoryFollower")
