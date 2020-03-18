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

#include <ignition/msgs/pose.pb.h>

#include <stack>
#include <vector>

#include <ignition/common/Profiler.hh>
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
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Model.hh"
#include "NewEntityPublisher.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private data class for NewEntityPublisher
class ignition::gazebo::systems::NewEntityPublisherPrivate
{
  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Entity publisher.
  public: transport::Node::Publisher entityPub;

  // /// \brief Keep the id of the world entity so we know how to traverse the
  // /// graph.
  // public: Entity worldEntity{kNullEntity};

  // /// \brief Keep the name of the world entity so it's easy to create temporary
  // /// scene graphs
  // public: std::string worldName;


  // /// \brief Initializes internal caches for entities whose poses are to be
  // /// published and their names
  // /// \param[in] _ecm Immutable reference to the entity component manager
  // public: void InitializeEntitiesToPublish(const EntityComponentManager &_ecm);

  // /// \brief Helper function to collect entity pose data
  // /// \param[in] _ecm Immutable reference to the entity component manager
  // public: void FillPoses(const EntityComponentManager &_ecm);

  // /// \brief Publishes poses collected by FillPoses with the provided time
  // /// stamp.
  // /// \param[in] _stampMsg Time stamp associated with published poses
  // public: void PublishPoses(const msgs::Time &_stampMsg);

  // /// \brief Ignition communication node.
  // public: transport::Node node;

  // /// \brief publisher for pose data
  // public: transport::Node::Publisher posePub;

  // /// \brief Model interface
  // public: Model model{kNullEntity};

  // /// \brief True to publish link pose
  // public: bool publishLinkPose = true;

  // /// \brief True to publish visual pose
  // public: bool publishVisualPose = false;

  // /// \brief True to publish collision pose
  // public: bool publishCollisionPose = false;

  // /// \brief True to publish sensor pose
  // public: bool publishSensorPose = false;

  // /// \brief True to publish nested model pose
  // public: bool publishNestedModelPose = false;

  // /// \brief Frequency of pose publications in Hz. A negative frequency
  // /// publishes as fast as possible (i.e, at the rate of the simulation step)
  // public: double updateFrequency = -1;

  // /// \brief Last time poses were published.
  // public: std::chrono::steady_clock::duration lastPosePubTime{0};

  // /// \brief Update period in nanoseconds calculated from the update_frequency
  // /// parameter
  // public: std::chrono::steady_clock::duration updatePeriod{0};

  // /// \brief Cache of entities, their frame names and their child frame names.
  // /// The key is the entity whose pose is to be published.
  // /// The frame name is the scoped name of the parent entity.
  // /// The child frame name is the scoped name of the entity (the key)
  // public: std::unordered_map<Entity, std::pair<std::string, std::string>>
  //             entitiesToPublish;

  // /// \brief A vector that contains the entities and their poses. This could
  // /// easily be a temporary, but having it as a member variable improves
  // /// performance by avoiding memory allocation
  // public: std::vector<std::pair<Entity, math::Pose3d>> poses;

  // /// \brief A variable that gets populated with poses. This also here as a
  // /// member variable to avoid repeated memory allocations and improve
  // /// performance.
  // public: ignition::msgs::Pose poseMsg;

  // /// \brief Whether cache variables have been initialized
  // public: bool initialized{false};
};

//////////////////////////////////////////////////
NewEntityPublisher::NewEntityPublisher()
  : dataPtr(std::make_unique<NewEntityPublisherPrivate>())
{
}

//////////////////////////////////////////////////
void NewEntityPublisher::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  // // World
  // const components::Name *name = _ecm.Component<components::Name>(_entity);
  // if (name == nullptr)
  // {
  //   ignerr << "World with id: " << _entity << " has no name. "
  //          << "NewEntityPublisher cannot create transport topics\n";
  //   return;
  // }

  // this->dataPtr->worldEntity = _entity;
  // this->dataPtr->worldName = name->Data();
}

//////////////////////////////////////////////////
void NewEntityPublisher::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("NewEntityPublisher::PostUpdate");

  // // \TODO(anyone) Support rewind
  // if (_info.dt < std::chrono::steady_clock::duration::zero())
  // {
  //   ignwarn << "Detected jump back in time ["
  //       << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
  //       << "s]. System may not work properly." << std::endl;
  // }

  // Nothing left to do if paused.
  // if (_info.paused)
  //   return;

  // Models
  _ecm.EachNew<components::Model, components::Name,
                   components::ParentEntity, components::Pose>(
      [&](const Entity &_entity, const components::Model *,
          const components::Name *_nameComp,
          const components::ParentEntity *_parentComp,
          const components::Pose *_poseComp) -> bool
      {
        //ignerr << _entity << std::endl;
        if (_nameComp && _poseComp)
        {
        //   auto modelMsg = std::make_shared<msgs::Model>();
        //   modelMsg->set_id(_entity);
        //   modelMsg->set_name(_nameComp->Data());
        //   modelMsg->mutable_pose()->CopyFrom(msgs::Convert(_poseComp->Data()));
  
          ignerr << _nameComp->Data() << std::endl;
        }
        return true;
      });
}

// //////////////////////////////////////////////////
// void NewEntityPublisherPrivate::InitializeEntitiesToPublish(
//     const EntityComponentManager &_ecm)
// {
//   std::stack<Entity> toCheck;
//   toCheck.push(this->model.Entity());
//   std::vector<Entity> visited;
//   while (!toCheck.empty())
//   {
//     Entity entity = toCheck.top();
//     toCheck.pop();
//     visited.push_back(entity);

//     auto link = _ecm.Component<components::Link>(entity);
//     auto nestedModel = _ecm.Component<components::Model>(entity);
//     auto visual = _ecm.Component<components::Visual>(entity);
//     auto collision = _ecm.Component<components::Collision>(entity);
//     auto sensor = _ecm.Component<components::Sensor>(entity);

//     bool fillPose = (link && this->publishLinkPose) ||
//         (nestedModel && this->publishNestedModelPose) ||
//         (visual && this->publishVisualPose) ||
//         (collision && this->publishCollisionPose) ||
//         (sensor && this->publishSensorPose);

//     if (fillPose)
//     {
//       std::string frame;
//       std::string childFrame;
//       auto entityName = _ecm.Component<components::Name>(entity);
//       if (!entityName)
//         continue;
//       childFrame =
//         removeParentScope(scopedName(entity, _ecm, "::", false), "::");

//       auto parent = _ecm.Component<components::ParentEntity>(entity);
//       if (parent)
//       {
//         auto parentName = _ecm.Component<components::Name>(parent->Data());
//         if (parentName)
//         {
//           frame = removeParentScope(
//               scopedName(parent->Data(), _ecm, "::", false), "::");
//         }
//       }
//       this->entitiesToPublish[entity] = std::make_pair(frame, childFrame);
//     }

//     // Recursively check if child entities need to be published
//     auto childEntities =
//         _ecm.ChildrenByComponents(entity, components::ParentEntity(entity));

//     // Use reverse iterators to match the order of entities found so as to match
//     // the expected order in the pose_publisher integration test.
//     for (auto childIt = childEntities.rbegin(); childIt != childEntities.rend();
//          ++childIt)
//     {
//       auto it = std::find(visited.begin(), visited.end(), *childIt);
//       if (it == visited.end())
//       {
//         // Only add to stack if the entity hasn't been already been visited.
//         // This also ensures there are no cycles.
//         toCheck.push(*childIt);
//       }
//     }
//   }

//   this->poses.reserve(this->entitiesToPublish.size());
// }

// //////////////////////////////////////////////////
// void NewEntityPublisherPrivate::FillPoses(const EntityComponentManager &_ecm)
// {
//   IGN_PROFILE("NewEntityPublisher::FillPose");
//   this->poses.clear();
//   for (const auto &entity : this->entitiesToPublish)
//   {
//     auto pose = _ecm.Component<components::Pose>(entity.first);
//     if (pose)
//       this->poses.emplace_back(entity.first, pose->Data());
//   }
// }

// //////////////////////////////////////////////////
// void NewEntityPublisherPrivate::PublishPoses(const msgs::Time &_stampMsg)
// {
//   IGN_PROFILE("NewEntityPublisher::PublishPoses");
//   // publish poses
//   for (const auto &[entity, pose] : this->poses)
//   {
//     auto entityIt = this->entitiesToPublish.find(entity);
//     if (entityIt == this->entitiesToPublish.end())
//       continue;
//     // fill pose msg
//     this->poseMsg.Clear();
//     // frame_id: parent entity name
//     // child_frame_id = entity name
//     // pose is the transform from frame_id to child_frame_id
//     auto header = this->poseMsg.mutable_header();
//     header->mutable_stamp()->CopyFrom(_stampMsg);
//     const std::string &frameId = entityIt->second.first;
//     const std::string &childFrameId = entityIt->second.second;
//     const math::Pose3d &transform = pose;
//     auto frame = header->add_data();
//     frame->set_key("frame_id");
//     frame->add_value(frameId);
//     auto childFrame = header->add_data();
//     childFrame->set_key("child_frame_id");
//     childFrame->add_value(childFrameId);

//     // set pose
//     this->poseMsg.set_name(childFrameId);
//     msgs::Set(&this->poseMsg, transform);
//     this->posePub.Publish(this->poseMsg);
//   }
// }

IGNITION_ADD_PLUGIN(NewEntityPublisher,
                    System,
                    NewEntityPublisher::ISystemConfigure,
                    NewEntityPublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(NewEntityPublisher,
                          "ignition::gazebo::systems::NewEntityPublisher")
