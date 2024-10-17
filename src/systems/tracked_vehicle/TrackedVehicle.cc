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

#include <gz/msgs/double.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
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

class gz::sim::systems::TrackedVehiclePrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const msgs::Twist &_msg);

  /// \brief Callback for steering efficiency subscription
  /// \param[in] _msg Steering efficiency message
  public: void OnSteeringEfficiency(const msgs::Double &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Gazebo communication node.
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

  /// \brief This variable is set to true each time a new command arrives.
  /// It is intended to be set to false after the command is processed.
  public: bool hasNewCommand{false};

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
    gzerr << "TrackedVehicle plugin should be attached to a model entity. "
    << "Failed to initialize." << std::endl;
    return;
  }

  const auto& modelName = this->dataPtr->model.Name(_ecm);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      _entity, components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  std::unordered_map<std::string, sdf::ElementConstPtr> tracks;

  if (_sdf->HasElement("body_link"))
    this->dataPtr->bodyLinkName = _sdf->Get<std::string>("body_link");

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("left_track");
  while (sdfElem)
  {
    const auto& linkName = sdfElem->Get<std::string>("link");
    this->dataPtr->leftTrackNames.push_back(linkName);
    tracks[linkName] = sdfElem;
    sdfElem = sdfElem->GetNextElement("left_track");
  }
  sdfElem = _sdf->FindElement("right_track");
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

    auto topic = validTopic({elem->Get<std::string>(
      "velocity_topic", prefix + "/track_cmd_vel").first});
    this->dataPtr->velPublishers[linkName] =
      this->dataPtr->node.Advertise<msgs::Double>(topic);

    topic = validTopic({elem->Get<std::string>("center_of_rotation_topic",
      prefix + "/track_cmd_center_of_rotation").first});
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

    auto sdf = _sdf->FindElement(tag);

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

  const auto kDefaultCmdVelTopic {topicPrefix + "/cmd_vel"};
  const auto topic = validTopic({
    _sdf->Get<std::string>("topic", kDefaultCmdVelTopic).first,
    kDefaultCmdVelTopic});

  this->dataPtr->node.Subscribe(topic, &TrackedVehiclePrivate::OnCmdVel,
      this->dataPtr.get());

  const auto kDefaultOdomTopic {topicPrefix + "/odometry"};
  const auto odomTopic = validTopic({
    _sdf->Get<std::string>("odom_topic", kDefaultOdomTopic).first,
    kDefaultOdomTopic});

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  const auto kDefaultTfTopic {topicPrefix + "/tf"};
  const auto tfTopic = validTopic({
    _sdf->Get<std::string>("tf_topic", kDefaultTfTopic).first,
    kDefaultTfTopic});

  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  const auto kDefaultSeTopic {topicPrefix + "/steering_efficiency"};
  const auto seTopic = validTopic({
    _sdf->Get<std::string>("steering_efficiency_topic", kDefaultSeTopic).first,
    kDefaultSeTopic});

  this->dataPtr->node.Subscribe(seTopic,
    &TrackedVehiclePrivate::OnSteeringEfficiency, this->dataPtr.get());

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  gzmsg << "TrackedVehicle [" << modelName << "] loaded:" << std::endl;
  gzmsg << "- tracks separation: " << this->dataPtr->tracksSeparation
         << " m" << std::endl;
  gzmsg << "- track height (for odometry): " << this->dataPtr->trackHeight
         << " m" << std::endl;
  gzmsg << "- initial steering efficiency: "
         << this->dataPtr->steeringEfficiency << std::endl;
  gzmsg << "- subscribing to twist messages on [" << topic << "]" << std::endl;
  gzmsg << "- subscribing to steering efficiency messages on ["
         << seTopic << "]" << std::endl;
  gzmsg << "- publishing odometry on [" << odomTopic << "]" << std::endl;
  gzmsg << "- publishing TF on [" << tfTopic << "]" << std::endl;

  // Initialize debugging helpers if needed
  this->dataPtr->debug = _sdf->Get<bool>("debug", false).first;
  if (this->dataPtr->debug)
  {
    this->dataPtr->debugMarker.set_ns(
      this->dataPtr->model.Name(_ecm) + "/cor");
    this->dataPtr->debugMarker.set_action(msgs::Marker::ADD_MODIFY);
    this->dataPtr->debugMarker.set_type(msgs::Marker::SPHERE);
    this->dataPtr->debugMarker.set_visibility(msgs::Marker::GUI);
    this->dataPtr->debugMarker.mutable_lifetime()->set_sec(0);
    this->dataPtr->debugMarker.mutable_lifetime()->set_nsec(4000000);
    this->dataPtr->debugMarker.set_id(1);

    // Set material properties
    msgs::Set(
      this->dataPtr->debugMarker.mutable_material()->mutable_ambient(),
      math::Color(0, 0, 1, 1));
    msgs::Set(
      this->dataPtr->debugMarker.mutable_material()->mutable_diffuse(),
      math::Color(0, 0, 1, 1));

    // Set marker scale
    msgs::Set(
      this->dataPtr->debugMarker.mutable_scale(),
      math::Vector3d(0.1, 0.1, 0.1));
  }
}

//////////////////////////////////////////////////
void TrackedVehicle::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("TrackedVehicle::PreUpdate");

  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
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
        gzwarn << "Failed to find body link [" << this->dataPtr->bodyLinkName
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
        gzwarn << "Failed to find left track [" << name << "] for model ["
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
        gzwarn << "Failed to find right track [" << name << "] for model ["
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
    gzmsg << "Found tracks for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }
}

//////////////////////////////////////////////////
void TrackedVehicle::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("TrackedVehicle::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void TrackedVehiclePrivate::UpdateOdometry(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("TrackedVehicle::UpdateOdometry");
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
  const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{
  GZ_PROFILE("TrackedVehicle::UpdateVelocity");

  // Read values protected by the mutex
  double linVel;
  double angVel;
  double steeringEfficiencyCopy;
  bool hadNewCommand;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
    steeringEfficiencyCopy = this->steeringEfficiency;
    hadNewCommand = this->hasNewCommand;
    this->hasNewCommand = false;
  }

  const auto dt = std::chrono::duration<double>(_info.dt).count();

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
    linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterAng->Limit(
    angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // decide whether commands to tracks should be sent
  bool sendCommandsToTracks{hadNewCommand};
  if (!hadNewCommand)
  {
    // if the speed limiter has been limiting the speed (or acceleration),
    // we let it saturate first and will stop publishing to tracks after that
    if (std::abs(linVel - this->last0Cmd.lin) > 1e-6)
    {
      sendCommandsToTracks = true;
    }
    else if (std::abs(angVel - this->last0Cmd.ang) > 1e-6)
    {
      sendCommandsToTracks = true;
    }
  }

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.ang = angVel;

  // only update and publish the following values when tracks should be
  // commanded with updated commands; none of these values changes when
  // linVel and angVel stay the same
  if (sendCommandsToTracks)
  {
    // Convert the target velocities to track velocities.
    this->rightSpeed = (linVel + angVel * this->tracksSeparation /
      (2.0 * steeringEfficiencyCopy));
    this->leftSpeed = (linVel - angVel * this->tracksSeparation /
      (2.0 * steeringEfficiencyCopy));

    // radius of the turn the robot is doing
    this->desiredRotationRadiusSigned =
      (fabs(angVel) < 1e-6) ?
      // is driving straight
      math::INF_D :
      (
        (fabs(linVel) < 1e-6) ?
        // is rotating about a single point
        0 :
        // general movement
        linVel / angVel);

    const auto bodyPose = worldPose(this->bodyLink, _ecm);
    const auto bodyYAxisGlobal =
      bodyPose.Rot().RotateVector(math::Vector3d(0, 1, 0));
    // centerOfRotation may be +inf
    this->centerOfRotation =
      (bodyYAxisGlobal * desiredRotationRadiusSigned) + bodyPose.Pos();

    for (const auto& track : this->leftTrackNames)
    {
      msgs::Double vel;
      vel.set_data(this->leftSpeed);
      this->velPublishers[track].Publish(vel);

      this->corPublishers[track].Publish(
        msgs::Convert(this->centerOfRotation));
    }

    for (const auto& track : this->rightTrackNames)
    {
      msgs::Double vel;
      vel.set_data(this->rightSpeed);
      this->velPublishers[track].Publish(vel);

      this->corPublishers[track].Publish(
        msgs::Convert(this->centerOfRotation));
    }
  }

  // Odometry is computed as if the vehicle were a diff-drive vehicle with
  // wheels as high as the tracks are.
  this->odomLeftWheelPos += this->leftSpeed / (this->trackHeight / 2) * dt;
  this->odomRightWheelPos += this->rightSpeed / (this->trackHeight / 2) * dt;

  if (this->debug)
  {
    gzdbg << "Tracked Vehicle " << this->model.Name(_ecm) << ":" << std::endl;
    gzdbg << "- cmd vel v=" << linVel << ", w=" << angVel
           << (hadNewCommand ? " (new command)" : "") << std::endl;
    gzdbg << "- left v=" << this->leftSpeed
           << ", right v=" << this->rightSpeed
           << (sendCommandsToTracks ? " (sent to tracks)" : "") << std::endl;

    msgs::Set(this->debugMarker.mutable_pose(), math::Pose3d(
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
  this->hasNewCommand = true;
}

//////////////////////////////////////////////////
void TrackedVehiclePrivate::OnSteeringEfficiency(
  const msgs::Double& _msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->steeringEfficiency = _msg.data();
  this->hasNewCommand = true;
}

GZ_ADD_PLUGIN(TrackedVehicle,
                    System,
                    TrackedVehicle::ISystemConfigure,
                    TrackedVehicle::ISystemPreUpdate,
                    TrackedVehicle::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(TrackedVehicle,
                          "gz::sim::systems::TrackedVehicle")
