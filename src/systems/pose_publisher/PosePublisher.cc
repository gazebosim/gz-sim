/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/msgs/pose.pb.h>

#include <stack>
#include <vector>

#include <sdf/Joint.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Model.hh"
#include "PosePublisher.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private data class for PosePublisher
class ignition::gazebo::systems::PosePublisherPrivate
{
  /// \brief Initializes internal caches for entities whose poses are to be
  /// published and their names
  /// \param[in] _ecm Immutable reference to the entity component manager
  public: void InitializeEntitiesToPublish(const EntityComponentManager &_ecm);

  /// \brief Helper function to collect entity pose data
  /// \param[in] _ecm Immutable reference to the entity component manager
  /// \param[out] _poses Pose vector to be filled
  /// \param[in] _static True to fill only static transforms,
  /// false to fill only dynamic transforms
  public: void FillPoses(const EntityComponentManager &_ecm,
      std::vector<std::pair<Entity, math::Pose3d>> &_poses,
      bool _static);

  /// \brief Publishes poses collected by FillPoses with the provided time
  /// stamp.
  /// \param[in] _poses Pose to publish
  /// \param[in] _stampMsg Time stamp associated with published poses
  /// \param[in] _publisher Publisher to publish the message
  public: void PublishPoses(
      std::vector<std::pair<Entity, math::Pose3d>> &_poses,
      const msgs::Time &_stampMsg,
      transport::Node::Publisher &_publisher);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief publisher for pose data
  public: transport::Node::Publisher posePub;

  /// \brief True to publish static transforms to a separate topic
  public: bool staticPosePublisher = false;

  /// \brief publisher for pose data
  public: transport::Node::Publisher poseStaticPub;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True to publish link pose
  public: bool publishLinkPose = true;

  /// \brief True to publish visual pose
  public: bool publishVisualPose = false;

  /// \brief True to publish collision pose
  public: bool publishCollisionPose = false;

  /// \brief True to publish sensor pose
  public: bool publishSensorPose = false;

  /// \brief True to publish nested model pose
  public: bool publishNestedModelPose = false;

  /// \brief Frequency of pose publications in Hz. A negative frequency
  /// publishes as fast as possible (i.e, at the rate of the simulation step)
  public: double updateFrequency = -1;

  /// \brief Last time poses were published.
  public: std::chrono::steady_clock::duration lastPosePubTime{0};

  /// \brief Last time static poses were published.
  public: std::chrono::steady_clock::duration lastStaticPosePubTime{0};

  /// \brief Update period in nanoseconds calculated from the update_frequency
  /// parameter
  public: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Update period in nanoseconds calculated from the
  /// static_update_frequency parameter
  public: std::chrono::steady_clock::duration staticUpdatePeriod{0};

  /// \brief Cache of entities, their frame names and their child frame names.
  /// The key is the entity whose pose is to be published.
  /// The frame name is the scoped name of the parent entity.
  /// The child frame name is the scoped name of the entity (the key)
  public: std::unordered_map<Entity, std::pair<std::string, std::string>>
              entitiesToPublish;

  /// \brief Entities with pose that can change over time, i.e. links connected
  /// by joints
  public: std::unordered_set<Entity> dynamicEntities;

  /// \brief A vector that contains the entities and their poses. This could
  /// easily be a temporary, but having it as a member variable improves
  /// performance by avoiding memory allocation
  public: std::vector<std::pair<Entity, math::Pose3d>> poses;

  /// \brief A vector that contains the entities and poses that are static.
  /// This could easily be a temporary, but having it as a member variable
  /// improves performance by avoiding memory allocation
  public: std::vector<std::pair<Entity, math::Pose3d>> staticPoses;

  /// \brief A variable that gets populated with poses. This also here as a
  /// member variable to avoid repeated memory allocations and improve
  /// performance.
  public: ignition::msgs::Pose poseMsg;

  /// \brief A variable that gets populated with poses. This also here as a
  /// member variable to avoid repeated memory allocations and improve
  /// performance.
  public: ignition::msgs::Pose_V poseVMsg;

  /// \brief True to publish a vector of poses. False to publish individual pose
  /// msgs.
  public: bool usePoseV = false;

  /// \brief Whether cache variables have been initialized
  public: bool initialized{false};
};

//////////////////////////////////////////////////
PosePublisher::PosePublisher()
  : dataPtr(std::make_unique<PosePublisherPrivate>())
{
}

//////////////////////////////////////////////////
void PosePublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "PosePublisher plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  // parse optional params
  this->dataPtr->publishLinkPose = _sdf->Get<bool>("publish_link_pose",
      this->dataPtr->publishLinkPose).first;

  this->dataPtr->publishNestedModelPose =
    _sdf->Get<bool>("publish_nested_model_pose",
        this->dataPtr->publishNestedModelPose).first;

  this->dataPtr->publishVisualPose =
    _sdf->Get<bool>("publish_visual_pose",
        this->dataPtr->publishVisualPose).first;

  this->dataPtr->publishCollisionPose =
    _sdf->Get<bool>("publish_collision_pose",
        this->dataPtr->publishCollisionPose).first;

  this->dataPtr->publishSensorPose =
    _sdf->Get<bool>("publish_sensor_pose",
        this->dataPtr->publishSensorPose).first;

  double updateFrequency = _sdf->Get<double>("update_frequency", -1).first;

  if (updateFrequency > 0)
  {
    std::chrono::duration<double> period{1 / updateFrequency};
    this->dataPtr->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  this->dataPtr->staticPosePublisher =
    _sdf->Get<bool>("static_publisher",
        this->dataPtr->staticPosePublisher).first;

  if (this->dataPtr->staticPosePublisher)
  {
    // update rate for static transforms. Default to same as <update_frequency>
    double staticPoseUpdateFrequency =
      _sdf->Get<double>("static_update_frequency", updateFrequency).first;

    if (staticPoseUpdateFrequency > 0)
    {
      std::chrono::duration<double> period{1 / staticPoseUpdateFrequency};
      this->dataPtr->staticUpdatePeriod =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          period);
    }
  }

  // create publishers
  this->dataPtr->usePoseV =
    _sdf->Get<bool>("use_pose_vector_msg", this->dataPtr->usePoseV).first;

  std::string poseTopic = scopedName(_entity, _ecm) + "/pose";
  std::string staticPoseTopic = poseTopic + "_static";

  if (this->dataPtr->usePoseV)
  {
    this->dataPtr->posePub =
      this->dataPtr->node.Advertise<ignition::msgs::Pose_V>(poseTopic);

    if (this->dataPtr->staticPosePublisher)
    {
      this->dataPtr->poseStaticPub =
          this->dataPtr->node.Advertise<ignition::msgs::Pose_V>(
          staticPoseTopic);
    }
  }
  else
  {
    this->dataPtr->posePub =
      this->dataPtr->node.Advertise<ignition::msgs::Pose>(poseTopic);
    if (this->dataPtr->staticPosePublisher)
    {
      this->dataPtr->poseStaticPub =
          this->dataPtr->node.Advertise<ignition::msgs::Pose>(
          staticPoseTopic);
    }
  }
}

//////////////////////////////////////////////////
void PosePublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("PosePublisher::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  bool publish = true;
  auto diff = _info.simTime - this->dataPtr->lastPosePubTime;
  // If the diff is positive and it's less than the update period, we skip
  // publication. If the diff is negative, then time has gone backward, we go
  // ahead publish and allow the time to be reset
  if ((diff > std::chrono::steady_clock::duration::zero()) &&
      (diff < this->dataPtr->updatePeriod))
  {
    publish = false;
  }

  bool publishStatic = true;
  auto staticDiff = _info.simTime - this->dataPtr->lastStaticPosePubTime;
  if (!this->dataPtr->staticPosePublisher ||
      ((staticDiff > std::chrono::steady_clock::duration::zero()) &&
      (staticDiff < this->dataPtr->staticUpdatePeriod)))
  {
    publishStatic = false;
  }

  if (!publish && !publishStatic)
    return;

  if (!this->dataPtr->initialized)
  {
    this->dataPtr->InitializeEntitiesToPublish(_ecm);
    this->dataPtr->initialized = true;
  }


  // if static transforms are published through a different topic
  if (this->dataPtr->staticPosePublisher)
  {
    if (publishStatic)
    {
      this->dataPtr->staticPoses.clear();
      this->dataPtr->FillPoses(_ecm, this->dataPtr->staticPoses, true);
      this->dataPtr->PublishPoses(this->dataPtr->staticPoses,
          convert<msgs::Time>(_info.simTime), this->dataPtr->poseStaticPub);
      this->dataPtr->lastStaticPosePubTime = _info.simTime;
    }

    if (publish)
    {
      this->dataPtr->poses.clear();
      this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, false);
      this->dataPtr->PublishPoses(this->dataPtr->poses,
          convert<msgs::Time>(_info.simTime), this->dataPtr->posePub);
      this->dataPtr->lastPosePubTime = _info.simTime;
    }
  }
  // publish all transforms to the same topic
  else if (publish)
  {
    this->dataPtr->poses.clear();
    this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, true);
    this->dataPtr->FillPoses(_ecm, this->dataPtr->poses, false);
    this->dataPtr->PublishPoses(this->dataPtr->poses,
        convert<msgs::Time>(_info.simTime), this->dataPtr->posePub);
    this->dataPtr->lastPosePubTime = _info.simTime;
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::InitializeEntitiesToPublish(
    const EntityComponentManager &_ecm)
{
  std::stack<Entity> toCheck;
  toCheck.push(this->model.Entity());
  std::vector<Entity> visited;
  while (!toCheck.empty())
  {
    Entity entity = toCheck.top();
    toCheck.pop();
    visited.push_back(entity);

    auto link = _ecm.Component<components::Link>(entity);
    auto nestedModel = _ecm.Component<components::Model>(entity);
    auto visual = _ecm.Component<components::Visual>(entity);
    auto collision = _ecm.Component<components::Collision>(entity);
    auto sensor = _ecm.Component<components::Sensor>(entity);
    auto joint = _ecm.Component<components::Joint>(entity);

    bool fillPose = (link && this->publishLinkPose) ||
        (nestedModel && this->publishNestedModelPose) ||
        (visual && this->publishVisualPose) ||
        (collision && this->publishCollisionPose) ||
        (sensor && this->publishSensorPose);

    if (fillPose)
    {
      std::string frame;
      std::string childFrame;
      auto entityName = _ecm.Component<components::Name>(entity);
      if (!entityName)
        continue;
      childFrame =
        removeParentScope(scopedName(entity, _ecm, "::", false), "::");

      auto parent = _ecm.Component<components::ParentEntity>(entity);
      if (parent)
      {
        auto parentName = _ecm.Component<components::Name>(parent->Data());
        if (parentName)
        {
          frame = removeParentScope(
              scopedName(parent->Data(), _ecm, "::", false), "::");
        }
      }
      this->entitiesToPublish[entity] = std::make_pair(frame, childFrame);
    }

    // get dynamic entities
    if (this->staticPosePublisher && joint)
    {
      sdf::JointType jointType =
          _ecm.Component<components::JointType>(entity)->Data();
      if (jointType != sdf::JointType::INVALID &&
          jointType != sdf::JointType::FIXED)
      {
        std::string parentLinkName =
            _ecm.Component<components::ParentLinkName>(entity)->Data();
        std::string childLinkName =
            _ecm.Component<components::ChildLinkName>(entity)->Data();

        auto parentLinkEntity = _ecm.EntityByComponents(
            components::Name(parentLinkName), components::Link(),
            components::ParentEntity(this->model.Entity()));
        auto childLinkEntity = _ecm.EntityByComponents(
            components::Name(childLinkName), components::Link(),
            components::ParentEntity(this->model.Entity()));

        // add to list if not a canonical link
        if (!_ecm.Component<components::CanonicalLink>(parentLinkEntity))
          this->dynamicEntities.insert(parentLinkEntity);
        if (!_ecm.Component<components::CanonicalLink>(childLinkEntity))
          this->dynamicEntities.insert(childLinkEntity);
      }
    }

    // Recursively check if child entities need to be published
    auto childEntities =
        _ecm.ChildrenByComponents(entity, components::ParentEntity(entity));

    // Use reverse iterators to match the order of entities found so as to match
    // the expected order in the pose_publisher integration test.
    for (auto childIt = childEntities.rbegin(); childIt != childEntities.rend();
         ++childIt)
    {
      auto it = std::find(visited.begin(), visited.end(), *childIt);
      if (it == visited.end())
      {
        // Only add to stack if the entity hasn't been already been visited.
        // This also ensures there are no cycles.
        toCheck.push(*childIt);
      }
    }
  }

  // sanity check to make sure dynamicEntities are a subset of entitiesToPublish
  for (auto const &ent : this->dynamicEntities)
  {
    if (this->entitiesToPublish.find(ent) == this->entitiesToPublish.end())
    {
      ignwarn << "Entity id: '" << ent << "' not found when creating a list "
              << "of dynamic entities in pose publisher." << std::endl;
    }
  }

  if (this->staticPosePublisher)
  {
    this->poses.reserve(this->dynamicEntities.size());
    this->staticPoses.reserve(
        this->entitiesToPublish.size() - this->dynamicEntities.size());
  }
  else
  {
    this->poses.reserve(this->entitiesToPublish.size());
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::FillPoses(const EntityComponentManager &_ecm,
    std::vector<std::pair<Entity, math::Pose3d>> &_poses, bool _static)
{
  IGN_PROFILE("PosePublisher::FillPose");

  for (const auto &entity : this->entitiesToPublish)
  {
    auto pose = _ecm.Component<components::Pose>(entity.first);
    if (!pose)
      continue;

    bool isStatic = this->dynamicEntities.find(entity.first) ==
          this->dynamicEntities.end();

    if (_static == isStatic)
      _poses.emplace_back(entity.first, pose->Data());
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::PublishPoses(
    std::vector<std::pair<Entity, math::Pose3d>> &_poses,
    const msgs::Time &_stampMsg,
    transport::Node::Publisher &_publisher)
{
  IGN_PROFILE("PosePublisher::PublishPoses");

  // publish poses
  ignition::msgs::Pose *msg = nullptr;
  if (this->usePoseV)
    this->poseVMsg.Clear();

  for (const auto &[entity, pose] : _poses)
  {
    auto entityIt = this->entitiesToPublish.find(entity);
    if (entityIt == this->entitiesToPublish.end())
      continue;

    if (this->usePoseV)
    {
      msg = this->poseVMsg.add_pose();
    }
    else
    {
      this->poseMsg.Clear();
      msg = &this->poseMsg;
    }

    // fill pose msg
    // frame_id: parent entity name
    // child_frame_id = entity name
    // pose is the transform from frame_id to child_frame_id
    IGN_ASSERT(msg != nullptr, "Pose msg is null");
    auto header = msg->mutable_header();

    header->mutable_stamp()->CopyFrom(_stampMsg);
    const std::string &frameId = entityIt->second.first;
    const std::string &childFrameId = entityIt->second.second;
    const math::Pose3d &transform = pose;
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(frameId);
    auto childFrame = header->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(childFrameId);

    // set pose
    msg->set_name(childFrameId);
    msgs::Set(msg, transform);

    // publish individual pose msgs
    if (!this->usePoseV)
      _publisher.Publish(this->poseMsg);
  }

  // publish pose vector msg
  if (this->usePoseV)
    _publisher.Publish(this->poseVMsg);
}

IGNITION_ADD_PLUGIN(PosePublisher,
                    System,
                    PosePublisher::ISystemConfigure,
                    PosePublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PosePublisher,
                          "ignition::gazebo::systems::PosePublisher")
