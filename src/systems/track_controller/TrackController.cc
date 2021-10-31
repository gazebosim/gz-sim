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

#include "TrackController.hh"

#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/marker.pb.h>
#include <ignition/msgs/Utility.hh>

#include <ignition/math/eigen3.hh>
#include <ignition/math/SpeedLimiter.hh>

#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::TrackControllerPrivate
{
  public : virtual ~TrackControllerPrivate() {}
  /// \brief Register a collision entity to work with this system (e.g. enable
  /// custom collision processing).
  /// \param[in] _ecm Entity Component Manager
  /// \param[in] _entity The collision to register
  /// \param[in] _link The link to which the collision belongs
  public: void RegisterCollision(EntityComponentManager& _ecm,
    const Entity& _entity, const Entity& _link);

  /// \brief Set velocity command to the track.
  /// \param[in] _msg The command.
  public: void OnCmdVel(const msgs::Double& _msg);

  /// \brief Set center of rotation command to the track.
  /// \param[in] _msg The command.
  public: void OnCenterOfRotation(const msgs::Vector3d& _msg);

  public: using P = physics::FeaturePolicy3d;
  public: using F = physics::SetContactJointPropertiesCallbackFeature;

  /// \brief The callback for CollectContactSurfaceProperties - all the magic
  /// happens here.
  /// \param[in] _collision1 The first colliding body.
  /// \param[in] _collision2 The second colliding body.
  /// \param[in] _point The contact point (in world coords).
  /// \param[in] _force Force in the contact point (may be omitted).
  /// \param[in] _normal Unit normal of the collision.
  /// \param[in] _depth Depth of penetration.
  /// \param[in] _numContactsOnCollision Number of contacts that share the same
  ///            collision body.
  /// \param[inout] _params The contact surface parameters to be set by this
  ///               system.
  public: void ComputeSurfaceProperties(
    const Entity& _collision1,
    const Entity& _collision2,
    const Eigen::Vector3d & _point,
    const Eigen::Vector3d * const _normal,
    F::ContactSurfaceParams<P>& _params);

  /// \brief Compute speed and direction of motion of the contact surface.
  /// \param[in] _beltSpeed Speed of the belt.
  /// \param[in] _beltDirection Direction of the belt (in world coords).
  /// \param[in] _frictionDirection First friction direction (in world coords).
  /// \return The computed contact surface speed.
  public: double ComputeSurfaceMotion(
    double _beltSpeed, const ignition::math::Vector3d &_beltDirection,
    const ignition::math::Vector3d &_frictionDirection);

  /// \brief Compute the first friction direction of the contact surface.
  /// \param[in] _centerOfRotation The point around which the track circles (
  ///                              +Inf vector in case of straight motion).
  /// \param[in] _contactWorldPosition Position of the contact point.
  /// \param[in] _contactNormal Normal of the contact surface (in world coords).
  /// \param[in] _beltDirection Direction of the belt (in world coords).
  public: ignition::math::Vector3d ComputeFrictionDirection(
    const ignition::math::Vector3d &_centerOfRotation,
    const ignition::math::Vector3d &_contactWorldPosition,
    const ignition::math::Vector3d &_contactNormal,
    const ignition::math::Vector3d &_beltDirection);

  /// \brief Name of the link to which the track is attached.
  public: std::string linkName;

  /// \brief Orientation of the track relative to the link. It is assumed that
  /// the track moves along the +x direction of the transformed coordinate
  /// system.
  public: math::Quaterniond trackOrientation;

  /// \brief Enables debugging prints and visualizations.
  public: bool debug {false};
  /// \brief Cached marker message for debugging purposes.
  public: msgs::Marker debugMarker;
  /// \brief ID of the debug marker. Should reset to 0 at each iteration start.
  public: uint64_t markerId;

  /// \brief Event manager.
  public: EventManager* eventManager;
  /// \brief Connection to CollectContactSurfaceProperties event.
  public: common::ConnectionPtr eventConnection;
  /// \brief Ignition transport node.
  public: transport::Node node;

  /// \brief The model this plugin is attached to.
  public: Model model;
  /// \brief Entity of the link this track is attached to.
  public: Entity linkEntity {kNullEntity};
  /// \brief Entities of all collision elements of the track's link.
  public: std::unordered_set<Entity> trackCollisions;

  /// \brief World pose of the track's link.
  public: math::Pose3d linkWorldPose;
  /// \brief World poses of all collision elements of the track's link.
  public: std::unordered_map<Entity, math::Pose3d> collisionsWorldPose;

  /// \brief The last commanded velocity.
  public: double velocity {0};
  /// \brief Commanded velocity clipped to allowable range.
  public: double limitedVelocity {0};
  /// \brief Previous clipped commanded velocity.
  public: double prevVelocity {0};
  /// \brief Second previous clipped commanded velocity.
  public: double prevPrevVelocity {0};
  /// \brief The point around which the track circles (in world coords). Should
  /// be set to +Inf if the track is going straight.
  public: math::Vector3d centerOfRotation {math::Vector3d::Zero * math::INF_D};
  /// \brief protects velocity and centerOfRotation
  public: std::mutex cmdMutex;

  /// \brief Limiter of the commanded velocity.
  public: math::SpeedLimiter limiter;
};

//////////////////////////////////////////////////
TrackController::TrackController()
  : dataPtr(std::make_unique<TrackControllerPrivate>())
{
}

//////////////////////////////////////////////////
TrackController::~TrackController()
{
}

//////////////////////////////////////////////////
void TrackController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->dataPtr->eventManager = &_eventMgr;

  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "TrackController should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  if (!_sdf->HasElement("link"))
  {
    ignerr << "TrackController plugin is missing <link> element." << std::endl;
    return;
  }
  this->dataPtr->linkName = _sdf->Get<std::string>("link");

  using P = physics::FeaturePolicy3d;
  using F = physics::SetContactJointPropertiesCallbackFeature;

  this->dataPtr->eventConnection = this->dataPtr->eventManager->
    Connect<events::CollectContactSurfaceProperties>(
    [this](
      const Entity& _collision1,
      const Entity& _collision2,
      const Eigen::Vector3d & _point,
      const Eigen::Vector3d * const /*_force*/,
      const Eigen::Vector3d * const _normal,
      const double * const /*_depth*/,
      const size_t /*_numContactsOnCollision*/,
      F::ContactSurfaceParams<P>& _params)
    {
      this->dataPtr->ComputeSurfaceProperties(_collision1, _collision2,
        _point, _normal, _params);
    }
  );

  _ecm.Each<components::Collision, components::Name, components::ParentEntity>(
    [&](const Entity & _collisionEntity,
      const components::Collision */*_collision*/,
      const components::Name */*_name*/,
      const components::ParentEntity *_parent)
    {
      this->dataPtr->RegisterCollision(_ecm, _collisionEntity, _parent->Data());
      return true;
    }
  );

  const auto topicPrefix = "/model/" + this->dataPtr->model.Name(_ecm) +
    "/link/" + this->dataPtr->linkName;

  const auto kDefaultVelTopic = topicPrefix + "/track_cmd_vel";
  const auto velTopic = _sdf->Get<std::string>(
    "velocity_topic", kDefaultVelTopic).first;
  if (!this->dataPtr->node.Subscribe(
    velTopic, &TrackControllerPrivate::OnCmdVel, this->dataPtr.get()))
  {
    ignerr << "Error subscribing to topic [" << velTopic << "]. "
           << "Track will not receive commands." << std::endl;
    return;
  }
  igndbg << "Subscribed to " << velTopic << " for receiving track velocity "
         << "commands." << std::endl;

  const auto kDefaultCorTopic = topicPrefix + "/track_cmd_center_of_rotation";
  const auto corTopic = _sdf->Get<std::string>(
    "center_of_rotation_topic", kDefaultCorTopic).first;
  if (!this->dataPtr->node.Subscribe(
    corTopic, &TrackControllerPrivate::OnCenterOfRotation,
    this->dataPtr.get()))
  {
    ignerr << "Error subscribing to topic [" << corTopic << "]. "
           << "Track will not receive center of rotation commands."
           << std::endl;
    return;
  }
  igndbg << "Subscribed to " << corTopic << " for receiving track center "
         << "of rotation commands." << std::endl;

  this->dataPtr->trackOrientation = _sdf->Get<math::Quaterniond>(
    "track_orientation", math::Quaterniond::Identity).first;

  auto hasVelocityLimits = false;
  auto hasAccelerationLimits = false;
  auto hasJerkLimits = false;
  auto minVel = std::numeric_limits<double>::lowest();
  auto maxVel = std::numeric_limits<double>::max();
  auto minAccel = std::numeric_limits<double>::lowest();
  auto maxAccel = std::numeric_limits<double>::max();
  auto minJerk = std::numeric_limits<double>::lowest();
  auto maxJerk = std::numeric_limits<double>::max();

  if (_sdf->HasElement("min_velocity"))
  {
    minVel = _sdf->Get<double>("min_velocity");
    hasVelocityLimits = true;
  }
  if (_sdf->HasElement("max_velocity"))
  {
    maxVel = _sdf->Get<double>("max_velocity");
    hasVelocityLimits = true;
  }
  if (_sdf->HasElement("min_acceleration"))
  {
    minAccel = _sdf->Get<double>("min_acceleration");
    hasAccelerationLimits = true;
  }
  if (_sdf->HasElement("max_acceleration"))
  {
    maxAccel = _sdf->Get<double>("max_acceleration");
    hasAccelerationLimits = true;
  }
  if (_sdf->HasElement("min_jerk"))
  {
    minJerk = _sdf->Get<double>("min_jerk");
    hasJerkLimits = true;
  }
  if (_sdf->HasElement("max_jerk"))
  {
    maxJerk = _sdf->Get<double>("max_jerk");
    hasJerkLimits = true;
  }

  if (hasVelocityLimits)
  {
    this->dataPtr->limiter.SetMinVelocity(minVel);
    this->dataPtr->limiter.SetMaxVelocity(maxVel);
  }
  if (hasAccelerationLimits)
  {
    this->dataPtr->limiter.SetMinAcceleration(minAccel);
    this->dataPtr->limiter.SetMaxAcceleration(maxAccel);
  }
  if (hasJerkLimits)
  {
    this->dataPtr->limiter.SetMinJerk(minJerk);
    this->dataPtr->limiter.SetMaxJerk(maxJerk);
  }

  this->dataPtr->debug = _sdf->Get<bool>("debug", false).first;
  if (this->dataPtr->debug)
  {
    this->dataPtr->debugMarker.set_ns(this->dataPtr->linkName + "/friction");
    this->dataPtr->debugMarker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->debugMarker.set_type(ignition::msgs::Marker::BOX);
    this->dataPtr->debugMarker.set_visibility(ignition::msgs::Marker::GUI);
    this->dataPtr->debugMarker.mutable_lifetime()->set_sec(0);
    this->dataPtr->debugMarker.mutable_lifetime()->set_nsec(4000000);

    // Set material properties
    ignition::msgs::Set(
      this->dataPtr->debugMarker.mutable_material()->mutable_ambient(),
      ignition::math::Color(0, 0, 1, 1));
    ignition::msgs::Set(
      this->dataPtr->debugMarker.mutable_material()->mutable_diffuse(),
      ignition::math::Color(0, 0, 1, 1));

    // Set marker scale
    ignition::msgs::Set(
      this->dataPtr->debugMarker.mutable_scale(),
      ignition::math::Vector3d(0.3, 0.03, 0.03));
  }
}

//////////////////////////////////////////////////
void TrackController::PreUpdate(
  const UpdateInfo& _info, EntityComponentManager& _ecm)
{
  _ecm.EachNew<components::Collision, components::Name,
               components::ParentEntity>(
    [&](const Entity & _entity,
      const components::Collision */*_collision*/,
      const components::Name */*_name*/,
      const components::ParentEntity *_parent)
    {
      this->dataPtr->RegisterCollision(_ecm, _entity, _parent->Data());
      return true;
    }
  );

  // Find link entity
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm,
      this->dataPtr->linkName);
  }
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    ignwarn << "Could not find track link [" << this->dataPtr->linkName << "]"
      << std::endl;
    return;
  }

  // Cache poses
  this->dataPtr->linkWorldPose = worldPose(this->dataPtr->linkEntity, _ecm);
  for (auto& entityPose : this->dataPtr->collisionsWorldPose)
    entityPose.second = worldPose(entityPose.first, _ecm);

  // Compute limited velocity command
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->cmdMutex);
    this->dataPtr->limitedVelocity = this->dataPtr->velocity;
  }
  this->dataPtr->limiter.Limit(
    this->dataPtr->limitedVelocity,  // in-out parameter
    this->dataPtr->prevVelocity,
    this->dataPtr->prevPrevVelocity, _info.dt);

  this->dataPtr->prevPrevVelocity = this->dataPtr->prevVelocity;
  this->dataPtr->prevVelocity = this->dataPtr->limitedVelocity;

  if (this->dataPtr->debug)
  {
    // Reset debug marker ID
    this->dataPtr->markerId = 1;
  }
}

//////////////////////////////////////////////////
void TrackControllerPrivate::ComputeSurfaceProperties(
  const Entity& _collision1,
  const Entity& _collision2,
  const Eigen::Vector3d & _point,
  const Eigen::Vector3d * const _normal,
  F::ContactSurfaceParams<P>& _params
  )
{
  using math::eigen3::convert;

  if (_normal == nullptr)
  {
    static bool informed = false;
    if (!informed)
    {
      ignerr << "TrackController requires a physics engine that computes "
             << "contact normals!" << std::endl;
      informed = true;
    }
    return;
  }

  const auto isCollision1Track = this->trackCollisions.find(_collision1) !=
    this->trackCollisions.end();
  const auto isCollision2Track = this->trackCollisions.find(_collision2) !=
    this->trackCollisions.end();
  if (!isCollision1Track && !isCollision2Track)
    return;

  const auto trackCollision = isCollision1Track ? _collision1 : _collision2;

  auto contactNormal = math::eigen3::convert(*_normal);

  const auto& collisionPose = this->collisionsWorldPose[trackCollision];

  // Flip the contact normal if it points outside the track collision
  if (contactNormal.Dot(collisionPose.Pos() - convert(_point)) < 0)
    contactNormal = -contactNormal;

  const auto trackWorldRot = this->linkWorldPose.Rot() * this->trackOrientation;
  const auto trackYAxisGlobal =
    trackWorldRot.RotateVector(math::Vector3d::UnitY);

  // Vector tangent to the belt pointing in the belt's movement direction
  // The belt's bottom moves backwards when the robot should move forward!
  auto beltDirection = contactNormal.Cross(trackYAxisGlobal);

  if (this->limitedVelocity < 0)
    beltDirection = -beltDirection;

  math::Vector3d cor;
  {
    std::lock_guard<std::mutex> lock(this->cmdMutex);
    cor = this->centerOfRotation;
  }

  const auto frictionDirection = this->ComputeFrictionDirection(
    cor, convert(_point), contactNormal, beltDirection);

  _params.Get<F::FirstFrictionalDirection<P>>().firstFrictionalDirection =
    convert(isCollision1Track ? frictionDirection : -frictionDirection);

  const auto surfaceMotion = this->ComputeSurfaceMotion(
    this->limitedVelocity, beltDirection, frictionDirection);

  _params.Get<F::ContactSurfaceMotionVelocity<P>>().
    contactSurfaceMotionVelocity.y() = surfaceMotion;

  if (this->debug)
  {
    igndbg << "Link: " << linkName << std::endl;
    igndbg << "- is collision 1 track " << (isCollision1Track ? "1" : "0")
           << std::endl;
    igndbg << "- velocity cmd         " << this->velocity << std::endl;
    igndbg << "- limited velocity cmd " << this->limitedVelocity << std::endl;
    igndbg << "- friction direction   " << frictionDirection << std::endl;
    igndbg << "- surface motion       " << surfaceMotion << std::endl;
    igndbg << "- contact point        " << convert(_point) << std::endl;
    igndbg << "- contact normal       " << contactNormal << std::endl;
    igndbg << "- track rot            " << trackWorldRot << std::endl;
    igndbg << "- track Y              " << trackYAxisGlobal << std::endl;
    igndbg << "- belt direction       " << beltDirection << std::endl;

    this->debugMarker.set_id(++this->markerId);

    math::Quaterniond rot;
    rot.From2Axes(math::Vector3d::UnitX, frictionDirection);
    math::Vector3d p = convert(_point);
    p += rot.RotateVector(
      math::Vector3d::UnitX * this->debugMarker.scale().x() / 2);

    ignition::msgs::Set(this->debugMarker.mutable_pose(), math::Pose3d(
      p.X(), p.Y(), p.Z(), rot.Roll(), rot.Pitch(), rot.Yaw()));
    this->debugMarker.mutable_material()->mutable_diffuse()->set_r(
      surfaceMotion >= 0 ? 0 : 1);

    this->node.Request("/marker", this->debugMarker);
  }
}

//////////////////////////////////////////////////
double TrackControllerPrivate::ComputeSurfaceMotion(
  const double _beltSpeed, const ignition::math::Vector3d &_beltDirection,
  const ignition::math::Vector3d &_frictionDirection)
{
  // the dot product <beltDirection,fdir1> is the cosine of the angle they
  // form (because both are unit vectors)
  // the belt should actually move in the opposite direction than is the desired
  // motion of the whole track - that's why the value is negated
  return -math::signum(_beltDirection.Dot(_frictionDirection)) *
         fabs(_beltSpeed);
}

//////////////////////////////////////////////////
ignition::math::Vector3d TrackControllerPrivate::ComputeFrictionDirection(
  const ignition::math::Vector3d &_centerOfRotation,
  const ignition::math::Vector3d &_contactWorldPosition,
  const ignition::math::Vector3d &_contactNormal,
  const ignition::math::Vector3d &_beltDirection)
{
  if (_centerOfRotation.IsFinite())
  {
    // non-straight drive

    // vector pointing from the center of rotation to the contact point
    const auto corToContact =
      (_contactWorldPosition - _centerOfRotation).Normalize();

    // the friction force should be perpendicular to corToContact
    auto frictionDirection = _contactNormal.Cross(corToContact);
    if (this->limitedVelocity < 0)
      frictionDirection = - frictionDirection;

    return frictionDirection;
  }
  else
  {
    // straight drive
    return _beltDirection;
  }
}

//////////////////////////////////////////////////
void TrackControllerPrivate::RegisterCollision(EntityComponentManager& _ecm,
  const Entity& _entity, const Entity& _link)
{
  if (this->linkEntity == kNullEntity)
    this->linkEntity = this->model.LinkByName(_ecm, this->linkName);

  if (_link != this->linkEntity)
    return;

  this->trackCollisions.insert(_entity);

  _ecm.SetComponentData<components::EnableContactSurfaceCustomization>(
    _entity, true);

  if (!_ecm.Component<components::WorldPose>(_entity))
    _ecm.SetComponentData<components::WorldPose>(_entity, math::Pose3d::Zero);
}

//////////////////////////////////////////////////
void TrackControllerPrivate::OnCmdVel(const msgs::Double& _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->velocity = _msg.data();
}

/////////////////////////////////////////////////
void TrackControllerPrivate::OnCenterOfRotation(const msgs::Vector3d& _msg)
{
  std::lock_guard<std::mutex> lock(this->cmdMutex);
  this->centerOfRotation = msgs::Convert(_msg);
}

IGNITION_ADD_PLUGIN(TrackController,
                    ignition::gazebo::System,
                    TrackController::ISystemConfigure,
                    TrackController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(TrackController,
                          "ignition::gazebo::systems::TrackController")
