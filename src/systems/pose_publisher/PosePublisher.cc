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
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/Model.hh"
#include "PosePublisher.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private data class for PosePublisher
class ignition::gazebo::systems::PosePublisherPrivate
{
  /// \brief Helper function to recursively collect entity pose data
  /// \param[in] _entity Entity whose data is to be collected
  /// \param[in] _ecm Immutable reference to the entity component manager
  /// \param[in] _poses Collection of pose data
  public: void FillPoses(const Entity &_entity,
    const EntityComponentManager &_ecm,
    std::vector<std::tuple<std::string, std::string, math::Pose3d>> &_poses);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief publisher for pose data
  public: transport::Node::Publisher posePub;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief True to publish link pose
  public: bool publishLinkPose = true;

  /// \brief True to publish visual pose
  public: bool publishVisualPose = false;

  /// \brief True to publish collision pose
  public: bool publishCollisionPose = false;

  /// \brief True to publish nested model pose
  public: bool publishNestedModelPose = false;
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

  std::string poseTopic =
    scopedName(_entity, _ecm) + "/pose";
  this->dataPtr->posePub =
    this->dataPtr->node.Advertise<ignition::msgs::Pose>(poseTopic);

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
}

//////////////////////////////////////////////////
void PosePublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // pose frame, child_frame, pose
  std::vector<std::tuple<std::string, std::string, math::Pose3d>> poses;
  this->dataPtr->FillPoses(this->dataPtr->model.Entity(), _ecm, poses);

  // publish poses
  auto simTimeSecNsec =
    ignition::math::durationToSecNsec(_info.simTime);
  for (const auto &pose : poses)
  {
    // fill pose msg
    // frame_id: parent entity name
    // child_frame_id = entity name
    // pose is the transform from frame_id to child_frame_id
    ignition::msgs::Pose poseMsg;
    auto header = poseMsg.mutable_header();
    header->mutable_stamp()->set_sec(simTimeSecNsec.first);
    header->mutable_stamp()->set_nsec(simTimeSecNsec.second);
    const std::string &frameId = std::get<0>(pose);
    const std::string &childFrameId = std::get<1>(pose);
    const math::Pose3d &transform = std::get<2>(pose);
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(frameId);
    auto childFrame = header->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(childFrameId);

    // set pose
    poseMsg.set_name(childFrameId);
    msgs::Set(&poseMsg, transform);
    this->dataPtr->posePub.Publish(poseMsg);
  }
}

//////////////////////////////////////////////////
void PosePublisherPrivate::FillPoses(const Entity &_entity,
    const EntityComponentManager &_ecm,
    std::vector<std::tuple<std::string, std::string, math::Pose3d>> &_poses)
{
  auto link = _ecm.Component<components::Link>(_entity);
  auto nestedModel = _ecm.Component<components::Model>(_entity);
  auto visual = _ecm.Component<components::Visual>(_entity);
  auto collision = _ecm.Component<components::Collision>(_entity);

  bool fillPose = (link && this->publishLinkPose) ||
      (nestedModel && this->publishNestedModelPose) ||
      (visual && this->publishVisualPose) ||
      (collision && this->publishCollisionPose);

  if (fillPose)
  {
    std::string frame;
    std::string childFrame;
    math::Pose3d transform;
    auto pose = _ecm.Component<components::Pose>(_entity);
    if (!pose)
      return;
    transform = pose->Data();

    // todo(anyone) use scopedName
    auto entityName = _ecm.Component<components::Name>(_entity);
    if (!entityName)
      return;
    childFrame = entityName->Data();

    // todo(anyone) use scopedName
    auto parent = _ecm.Component<components::ParentEntity>(_entity);
    if (parent)
    {
      auto parentName = _ecm.Component<components::Name>(parent->Data());
      if (parentName)
        frame = parentName->Data();
    }
    auto p = std::make_tuple(frame, childFrame, transform);
    _poses.push_back(p);
  }

  auto childEntities = _ecm.ChildrenByComponents(_entity,
      components::ParentEntity(_entity));
  for (const auto childEntity : childEntities)
    this->FillPoses(childEntity, _ecm, _poses);
}

IGNITION_ADD_PLUGIN(PosePublisher,
                    System,
                    PosePublisher::ISystemConfigure,
                    PosePublisher::ISystemPostUpdate)
