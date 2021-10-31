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

#include "TrackedVehicle.hh"

#include <ignition/msgs/odometry.pb.h>

#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/math/DiffDriveOdometry.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/SpeedLimiter.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity.
  double lin {0.0};

  /// \brief Angular velocity.
  double ang {0.0};

  Commands() {}
};

class ignition::gazebo::systems::TrackedVehiclePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const ignition::msgs::Twist &_msg);

  /// \brief Callback for steering efficiency subscription
  /// \param[in] _msg Steering efficiency message
  public: void OnSteeringEfficiency(const ignition::msgs::Double &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief The link of the vehicle body (should be between left and right
  /// tracks, center of this link will be the center of rotation).
  public: Entity bodyLink {kNullEntity};

  /// \brief Entities of the left tracks
  public: std::vector<Entity> leftTracks;

  /// \brief Entities of the right tracks
  public: std::vector<Entity> rightTracks;

  /// \brief Name of the body link
  public: std::string bodyLinkName;

  /// \brief Names of left tracks
  public: std::vector<std::string> leftTrackNames;

  /// \brief Names of right tracks
  public: std::vector<std::string> rightTrackNames;

  /// \brief Velocity publishers of tracks.
  public: std::unordered_map<std::string, transport::Node::Publisher>
    velPublishers;

  /// \brief Center of rotation publishers of tracks.
  public: std::unordered_map<std::string, transport::Node::Publisher>
    corPublishers;

  /// \brief Calculated speed of left tracks
  public: double leftSpeed{0};

  /// \brief Calculated speed of right tracks
  public: double rightSpeed{0};

  /// \brief Radius of the desired rotation (rad).
  public: double desiredRotationRadiusSigned {0};

  /// \brief Fake position encoder of left track (for computing odometry).
  public: math::Angle odomLeftWheelPos {0};

  /// \brief Fake position encoder of left track (for computing odometry).
  public: math::Angle odomRightWheelPos {0};

  /// \brief The point around which the vehicle should circle (in world coords).
  public: math::Vector3d centerOfRotation {0, 0, 0};

  /// \brief Distance between tracks.
  public: double tracksSeparation{1.0};

  /// \brief Height of the tracks.
  public: double trackHeight{0.2};

  /// \brief Steering efficiency.
  public: double steeringEfficiency{0.5};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Update period calculated from <odom_publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Diff drive odometry.
  public: math::DiffDriveOdometry odom;

  /// \brief Diff drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Diff drive tf message publisher.
  public: transport::Node::Publisher tfPub;

  /// \brief Linear velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;

  /// \brief Enables debugging prints and visualizations.
  public: bool debug {false};

  /// \brief Cached marker message for debugging purposes.
  public: msgs::Marker debugMarker;
};

//////////////////////////////////////////////////
TrackedVehicle::TrackedVehicle()
  : dataPtr(std::make_unique<TrackedVehiclePrivate>())
{
}

//////////////////////////////////////////////////
TrackedVehicle::~TrackedVehicle()
{
}

//////////////////////////////////////////////////
void TrackedVehicle::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "TrackedVehicle plugin should be attached to a model entity. "
    << "Failed to initialize." << std::endl;
    return;
  }

  const auto& modelName = this->dataPtr->model.Name(_ecm);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      _entity, components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  std::unordered_map<std::string, sdf::ElementPtr> tracks;

  if (_sdf->HasElement("body_link"))
    this->dataPtr->bodyLinkName = _sdf->Get<std::string>("body_link");

  // Get params from SDF
  sdf::ElementPtr sdfElem = ptr->GetElement("left_track");
  while (sdfElem)
  {
    const auto& linkName = sdfElem->Get<std::string>("link");
    this->dataPtr->leftTrackNames.push_back(linkName);
    tracks[linkName] = sdfElem;
    sdfElem = sdfElem->GetNextElement("left_track");
  }
  sdfElem = ptr->GetElement("right_track");
  while (sdfElem)
  {
    const auto& linkName = sdfElem->Get<std::string>("link");
    this->dataPtr->rightTrackNames.push_back(linkName);
    tracks[linkName] = sdfElem;
    sdfElem = sdfElem->GetNextElement("right_track");
  }

  for (const auto &[linkName, elem] : tracks)
  {
    const auto prefix = "/model/" + modelName + "/link/" + linkName;

    auto topic = elem->Get<std::string>(
      "velocity_topic", prefix + "/track_cmd_vel").first;
    this->dataPtr->velPublishers[linkName] =
      this->dataPtr->node.Advertise<msgs::Double>(topic);

    topic = elem->Get<std::string>("center_of_rotation_topic",
      prefix + "/track_cmd_center_of_rotation").first;
    this->dataPtr->corPublishers[linkName] =
      this->dataPtr->node.Advertise<msgs::Vector3d>(topic);
  }

  this->dataPtr->tracksSeparation = _sdf->Get<double>("tracks_separation",
      this->dataPtr->tracksSeparation).first;
  this->dataPtr->steeringEfficiency = _sdf->Get<double>("steering_efficiency",
      this->dataPtr->steeringEfficiency).first;

  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();
  this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();

  std::map<std::string, math::SpeedLimiter *> limits = {
    {"linear_velocity", this->dataPtr->limiterLin.get()},
    {"angular_velocity", this->dataPtr->limiterAng.get()},
  };

  for (auto& [tag, limiter] : limits)
  {
    if (!_sdf->HasElement(tag))
      continue;

    auto sdf = ptr->GetElement(tag);

    // Parse speed limiter parameters.
    bool hasVelocityLimits     = false;
    bool hasAccelerationLimits = false;
    bool hasJerkLimits         = false;
    double minVel              = std::numeric_limits<double>::lowest();
    double maxVel              = std::numeric_limits<double>::max();
    double minAccel            = std::numeric_limits<double>::lowest();
    double maxAccel            = std::numeric_limits<double>::max();
    double minJerk             = std::numeric_limits<double>::lowest();
    double maxJerk             = std::numeric_limits<double>::max();

    if (sdf->HasElement("min_velocity"))
    {
      minVel = sdf->Get<double>("min_velocity");
      hasVelocityLimits = true;
    }
    if (sdf->HasElement("max_velocity"))
    {
      maxVel = sdf->Get<double>("max_velocity");
      hasVelocityLimits = true;
    }
    if (sdf->HasElement("min_acceleration"))
    {
      minAccel = sdf->Get<double>("min_acceleration");
      hasAccelerationLimits = true;
    }
    if (sdf->HasElement("max_acceleration"))
    {
      maxAccel = sdf->Get<double>("max_acceleration");
      hasAccelerationLimits = true;
    }
    if (sdf->HasElement("min_jerk"))
    {
      minJerk = sdf->Get<double>("min_jerk");
      hasJerkLimits = true;
    }
    if (sdf->HasElement("max_jerk"))
    {
      maxJerk = sdf->Get<double>("max_jerk");
      hasJerkLimits = true;
    }

    if (hasVelocityLimits)
    {
      limiter->SetMinVelocity(minVel);
      limiter->SetMaxVelocity(maxVel);
    }

    if (hasAccelerationLimits)
    {
      limiter->SetMinAcceleration(minAccel);
      limiter->SetMaxAcceleration(maxAccel);
    }

    if (hasJerkLimits)
    {
      limiter->SetMinJerk(minJerk);
      limiter->SetMaxJerk(maxJerk);
    }
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->tracksSeparation,
      this->dataPtr->trackHeight/2, this->dataPtr->trackHeight/2);

  // Subscribe to commands
  const auto topicPrefix = "/model/" + this->dataPtr->model.Name(_ecm);

  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back(topicPrefix + "/cmd_vel");
  const auto topic = validTopic(topics);

  this->dataPtr->node.Subscribe(topic, &TrackedVehiclePrivate::OnCmdVel,
      this->dataPtr.get());

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back(topicPrefix + "/odometry");
  const auto odomTopic = validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  std::vector<std::string> tfTopics;
  if (_sdf->HasElement("tf_topic"))
  {
    tfTopics.push_back(_sdf->Get<std::string>("tf_topic"));
  }
  tfTopics.push_back(topicPrefix + "/tf");
  const auto tfTopic = validTopic(tfTopics);

  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  std::vector<std::string> seTopics;
  if (_sdf->HasElement("steering_efficiency_topic"))
  {
    seTopics.push_back(
      _sdf->Get<std::string>("steering_efficiency_topic"));
  }
  seTopics.push_back(topicPrefix + "/steering_efficiency");
  const auto seTopic = validTopic(seTopics);

  this->dataPtr->node.Subscribe(seTopic,
    &TrackedVehiclePrivate::OnSteeringEfficiency, this->dataPtr.get());

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  ignmsg << "TrackedVehicle [" << modelName << "] loaded:" << std::endl;
  ignmsg << "- tracks separation: " << this->dataPtr->tracksSeparation
         << " m" << std::endl;
  ignmsg << "- track height (for odometry): " << this->dataPtr->trackHeight
         << " m" << std::endl;
  ignmsg << "- initial steering efficiency: "
         << this->dataPtr->steeringEfficiency << std::endl;
  ignmsg << "- subscribing to twist messages on [" << topic << "]" << std::endl;
  ignmsg << "- subscribing to steering efficiency messages on ["
         << seTopic << "]" << std::endl;
  ignmsg << "- publishing odometry on [" << odomTopic << "]" << std::endl;
  ignmsg << "- publishing TF on [" << tfTopic << "]" << std::endl;

  // Initialize debugging helpers if needed
  this->dataPtr->debug = _sdf->Get<bool>("debug", false).first;
  if (this->dataPtr->debug)
  {
    this->dataPtr->debugMarker.set_ns(
      this->dataPtr->model.Name(_ecm) + "/cor");
    this->dataPtr->debugMarker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    this->dataPtr->debugMarker.set_type(ignition::msgs::Marker::SPHERE);
    this->dataPtr->debugMarker.set_visibility(ignition::msgs::Marker::GUI);
    this->dataPtr->debugMarker.mutable_lifetime()->set_sec(0);
    this->dataPtr->debugMarker.mutable_lifetime()->set_nsec(4000000);
    this->dataPtr->debugMarker.set_id(1);

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
      ignition::math::Vector3d(0.1, 0.1, 0.1));
  }
}

//////////////////////////////////////////////////
void TrackedVehicle::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrackedVehicle::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. Resetting odometry." << std::endl;
    this->dataPtr->odom.Init(
      std::chrono::steady_clock::time_point(_info.simTime));
  }

  // If the links haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);

  if (this->dataPtr->bodyLink == kNullEntity)
  {
    if (!this->dataPtr->bodyLinkName.empty())
      this->dataPtr->bodyLink =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->bodyLinkName);
    else
      this->dataPtr->bodyLink = this->dataPtr->canonicalLink.Entity();

    if (this->dataPtr->bodyLink == kNullEntity)
    {
      static bool warned {false};
      if (!warned)
      {
        ignwarn << "Failed to find body link [" << this->dataPtr->bodyLinkName
          << "] for model [" << modelName << "]" << std::endl;
        warned = true;
      }
      return;
    }
  }

  if (this->dataPtr->leftTracks.empty() ||
      this->dataPtr->rightTracks.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->leftTrackNames)
    {
      Entity track = this->dataPtr->model.LinkByName(_ecm, name);
      if (track != kNullEntity)
        this->dataPtr->leftTracks.push_back(track);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find left track [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->rightTrackNames)
    {
      Entity track = this->dataPtr->model.LinkByName(_ecm, name);
      if (track != kNullEntity)
        this->dataPtr->rightTracks.push_back(track);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        ignwarn << "Failed to find right track [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->leftTracks.empty() || this->dataPtr->rightTracks.empty())
    return;

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    ignmsg << "Found tracks for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;
}

//////////////////////////////////////////////////
void TrackedVehicle::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrackedVehicle::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void TrackedVehiclePrivate::UpdateOdometry(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrackedVehicle::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  if (this->leftTracks.empty() || this->rightTracks.empty())
    return;

  this->odom.Update(this->odomLeftWheelPos, this->odomRightWheelPos,
      std::chrono::steady_clock::time_point(_info.simTime));

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
  msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());

  math::Quaterniond orientation(0, 0, *this->odom.Heading());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
  msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  if (this->sdfChildFrameId.empty())
  {
    if (!this->bodyLinkName.empty())
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + this->bodyLinkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tfMsg;
  auto *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the messages
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void TrackedVehiclePrivate::UpdateVelocity(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("TrackedVehicle::UpdateVelocity");

  // Read values protected by the mutex
  double linVel;
  double angVel;
  double steeringEfficiencyCopy;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
    steeringEfficiencyCopy = this->steeringEfficiency;
  }

  const auto dt = std::chrono::duration<double>(_info.dt).count();

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
    linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterAng->Limit(
    angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.ang = angVel;

  // Convert the target velocities to track velocities.
  this->rightSpeed =
    (linVel + angVel * this->tracksSeparation / (2.0 * steeringEfficiencyCopy));
  this->leftSpeed =
    (linVel - angVel * this->tracksSeparation / (2.0 * steeringEfficiencyCopy));

  // Odometry is computed as if the vehicle were a diff-drive vehicle with
  // wheels as high as the tracks are.
  this->odomLeftWheelPos += this->leftSpeed / (this->trackHeight / 2) * dt;
  this->odomRightWheelPos += this->rightSpeed / (this->trackHeight / 2) * dt;

  // radius of the turn the robot is doing
  this->desiredRotationRadiusSigned =
    (fabs(angVel) < 0.1) ?
      // is driving straight
      math::INF_D :
      (
        (fabs(linVel) < 0.1) ?
          // is rotating about a single point
          0 :
          // general movement
          linVel / angVel);

  const auto bodyPose = worldPose(this->bodyLink, _ecm);
  const auto bodyYAxisGlobal =
    bodyPose.Rot().RotateVector(ignition::math::Vector3d(0, 1, 0));
  // centerOfRotation may be +inf
  this->centerOfRotation =
    (bodyYAxisGlobal * desiredRotationRadiusSigned) + bodyPose.Pos();

  for (const auto& track : this->leftTrackNames)
  {
    msgs::Double vel;
    vel.set_data(this->leftSpeed);
    this->velPublishers[track].Publish(vel);

    this->corPublishers[track].Publish(msgs::Convert(this->centerOfRotation));
  }

  for (const auto& track : this->rightTrackNames)
  {
    msgs::Double vel;
    vel.set_data(this->rightSpeed);
    this->velPublishers[track].Publish(vel);

    this->corPublishers[track].Publish(msgs::Convert(this->centerOfRotation));
  }

  if (this->debug)
  {
    igndbg << "Tracked Vehicle " << this->model.Name(_ecm) << ":" << std::endl;
    igndbg << "- cmd vel v=" << linVel << ", w=" << angVel << std::endl;
    igndbg << "- left v=" << this->leftSpeed << ", right v=" << this->rightSpeed
           << std::endl;

    ignition::msgs::Set(this->debugMarker.mutable_pose(), math::Pose3d(
      this->centerOfRotation.X(),
      this->centerOfRotation.Y(),
      this->centerOfRotation.Z(),
      0, 0, 0));
    this->node.Request("/marker", this->debugMarker);
  }
}

//////////////////////////////////////////////////
void TrackedVehiclePrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}

//////////////////////////////////////////////////
void TrackedVehiclePrivate::OnSteeringEfficiency(
  const ignition::msgs::Double& _msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->steeringEfficiency = _msg.data();
}

IGNITION_ADD_PLUGIN(TrackedVehicle,
                    ignition::gazebo::System,
                    TrackedVehicle::ISystemConfigure,
                    TrackedVehicle::ISystemPreUpdate,
                    TrackedVehicle::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(TrackedVehicle,
                          "ignition::gazebo::systems::TrackedVehicle")
