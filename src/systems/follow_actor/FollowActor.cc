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

#include <string>

#include <gz/plugin/Register.hh>

#include <gz/common/Profiler.hh>

#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "FollowActor.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private FollowActor data class.
class gz::sim::systems::FollowActorPrivate
{
  /// \brief Entity for the actor.
  public: Entity actorEntity{kNullEntity};

  /// \brief Velocity of the actor
  public: double velocity{0.8};

  /// \brief Current target to follow
  public: Entity targetEntity{kNullEntity};

  /// \brief Minimum distance in meters to keep away from target.
  public: double minDistance{1.2};

  /// \brief Maximum distance in meters to keep away from target.
  public: double maxDistance{4};

  /// \brief Velocity of the animation dislocation on the X axis, in m/s.
  /// Used to coordinate translational motion with the actor's feet.
  /// TODO(louise) Automatically calculate it from the root node's first and
  /// last keyframes
  public: double animationXVel{2.0};

  /// \brief Time of the last update.
  public: std::chrono::steady_clock::duration lastUpdate{0};

  /// \brief True if currently following
  public: bool following{true};
};

//////////////////////////////////////////////////
FollowActor::FollowActor() :
  System(), dataPtr(std::make_unique<FollowActorPrivate>())
{
}

//////////////////////////////////////////////////
FollowActor::~FollowActor() = default;

//////////////////////////////////////////////////
void FollowActor::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->actorEntity = _entity;

  auto actorComp = _ecm.Component<components::Actor>(_entity);
  if (!actorComp)
  {
    gzerr << "Entity [" << _entity << "] is not an actor." << std::endl;
    return;
  }

  if (!_sdf->HasElement("target"))
  {
    gzerr << "Missing <target>, can't follow." << std::endl;
    return;
  }

  auto targetName = _sdf->Get<std::string>("target");
  this->dataPtr->targetEntity = _ecm.EntityByComponents(components::Name(
      targetName));
  if (kNullEntity == this->dataPtr->targetEntity)
  {
    gzerr << "Failed to find target entity [" << targetName << "]"
           << std::endl;
    return;
  }

  if (_sdf->HasElement("velocity"))
    this->dataPtr->velocity = _sdf->Get<double>("velocity");

  if (_sdf->HasElement("min_distance"))
    this->dataPtr->minDistance = _sdf->Get<double>("min_distance");

  if (_sdf->HasElement("max_distance"))
    this->dataPtr->maxDistance = _sdf->Get<double>("max_distance");

  if (_sdf->HasElement("animation_x_vel"))
    this->dataPtr->animationXVel = _sdf->Get<double>("animation_x_vel");

  std::string animationName;

  // If animation not provided, use first one from SDF
  if (!_sdf->HasElement("animation"))
  {
    if (actorComp->Data().AnimationCount() < 1)
    {
      gzerr << "Actor SDF doesn't have any animations." << std::endl;
      return;
    }

    animationName = actorComp->Data().AnimationByIndex(0)->Name();
  }
  else
  {
    animationName = _sdf->Get<std::string>("animation");
  }

  if (animationName.empty())
  {
    gzerr << "Can't find actor's animation name." << std::endl;
    return;
  }

  auto animationNameComp = _ecm.Component<components::AnimationName>(_entity);
  if (nullptr == animationNameComp)
  {
    _ecm.CreateComponent(_entity, components::AnimationName(animationName));
  }
  else
  {
    *animationNameComp = components::AnimationName(animationName);
  }
  // Mark as a one-time-change so that the change is propagated to the GUI
  _ecm.SetChanged(_entity,
      components::AnimationName::typeId, ComponentState::OneTimeChange);

  // Set custom animation time from this plugin
  auto animTimeComp = _ecm.Component<components::AnimationTime>(_entity);
  if (nullptr == animTimeComp)
  {
    _ecm.CreateComponent(_entity, components::AnimationTime());
  }

  math::Pose3d initialPose;
  auto poseComp = _ecm.Component<components::Pose>(_entity);
  if (nullptr == poseComp)
  {
    _ecm.CreateComponent(_entity, components::Pose(
        math::Pose3d::Zero));
  }
  else
  {
    initialPose = poseComp->Data();

    // We'll be setting the actor's X/Y pose with respect to the world. So we
    // zero the current values.
    auto newPose = initialPose;
    newPose.Pos().X(0);
    newPose.Pos().Y(0);
    *poseComp = components::Pose(newPose);
  }

  // Having a trajectory pose prevents the actor from moving with the
  // SDF script
  auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(_entity);
  if (nullptr == trajPoseComp)
  {
    // Leave Z to the pose component, control only 2D with Trajectory
    initialPose.Pos().Z(0);
    _ecm.CreateComponent(_entity, components::TrajectoryPose(initialPose));
  }
}

//////////////////////////////////////////////////
void FollowActor::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("FollowActor::PreUpdate");

  if (_info.paused)
    return;

  // TODO(louise) Throttle this system

  // Time delta
  std::chrono::duration<double> dtDuration = _info.simTime -
      this->dataPtr->lastUpdate;
  double dt = dtDuration.count();

  this->dataPtr->lastUpdate = _info.simTime;

  // Is there a follow target?
  if (this->dataPtr->targetEntity == kNullEntity)
    return;

  // Current world pose
  auto trajPoseComp = _ecm.Component<components::TrajectoryPose>(
      this->dataPtr->actorEntity);
  auto actorPose = trajPoseComp->Data();
  auto initialPose = actorPose;

  // Current target
  auto targetPose = _ecm.Component<components::Pose>(
      this->dataPtr->targetEntity)->Data();

  // Direction to target
  auto dir = targetPose.Pos() - actorPose.Pos();
  dir.Z(0);

  // Stop if too close to target
  if (dir.Length() <= this->dataPtr->minDistance)
  {
    return;
  }

  // Stop following if too far from target
  if (dir.Length() > this->dataPtr->maxDistance)
  {
    if (this->dataPtr->following)
    {
      gzmsg << "Target [" << this->dataPtr->targetEntity
             <<  "] too far, actor [" << this->dataPtr->actorEntity
             <<"] stopped following" << std::endl;
      this->dataPtr->following = false;
    }
    return;
  }
  if (!this->dataPtr->following)
  {
    gzmsg << "Target [" << this->dataPtr->targetEntity
           <<  "] within range, actor [" << this->dataPtr->actorEntity
           <<"] started following" << std::endl;
    this->dataPtr->following = true;
  }

  dir.Normalize();

  // Towards target
  math::Angle yaw = atan2(dir.Y(), dir.X());
  yaw.Normalize();

  actorPose.Pos() += dir * this->dataPtr->velocity * dt;
  actorPose.Pos().Z(0);
  actorPose.Rot() = math::Quaterniond(0, 0, yaw.Radian());

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (actorPose.Pos() - initialPose.Pos()).Length();

  // Update actor root pose
  *trajPoseComp = components::TrajectoryPose(actorPose);
  // Mark as a one-time-change so that the change is propagated to the GUI
  _ecm.SetChanged(this->dataPtr->actorEntity,
      components::TrajectoryPose::typeId, ComponentState::OneTimeChange);

  // Update actor bone trajectories based on animation time
  auto animTimeComp = _ecm.Component<components::AnimationTime>(
      this->dataPtr->actorEntity);

  auto animTime = animTimeComp->Data() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(distanceTraveled *
    this->dataPtr->animationXVel));
  *animTimeComp = components::AnimationTime(animTime);
  // Mark as a one-time-change so that the change is propagated to the GUI
  _ecm.SetChanged(this->dataPtr->actorEntity,
      components::AnimationTime::typeId, ComponentState::OneTimeChange);
}

GZ_ADD_PLUGIN(FollowActor, System,
  FollowActor::ISystemConfigure,
  FollowActor::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(FollowActor, "gz::sim::systems::FollowActor")
