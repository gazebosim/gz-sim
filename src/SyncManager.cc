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

#include <sdf/Geometry.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/World.hh>

#include "ignition/common/Profiler.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerActive.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/World.hh"

#include "SyncManager.hh"
#include "SimulationRunner.hh"

#include "network/NetworkManagerPrimary.hh"
#include "network/NetworkManagerSecondary.hh"

#include "msgs/performer_affinity.pb.h"


using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
SyncManager::SyncManager(
  SimulationRunner * _runner, bool _useLevels,
  bool _useDistSim)
: runner(_runner),
  useLevels(_useLevels),
  useDistSim(_useDistSim)
{
  this->entityCreator = std::make_unique<SdfEntityCreator>(
    this->runner->entityCompMgr,
    this->runner->eventMgr);

  if (!_useDistSim) {
    igndbg << "Distributed Sim off, SyncManager disabled." << std::endl;
    return;
  }

  if (_useDistSim && !this->runner->networkMgr) {
    ignerr << "Cannot start distributed simulation. " <<
      "Server was given --distributed flag, " <<
      "but invalid configuration detected." << std::endl;
    return;
  }

  this->role = this->runner->networkMgr->Role();

  if (this->role != NetworkRole::None) {
    this->posePub = this->node.Advertise<ignition::msgs::Pose_V>("pose_update");
  }

  if (this->role == NetworkRole::SimulationPrimary) {
    this->node.Subscribe("pose_update", &SyncManager::OnPose, this);
  }

  this->worldEntity = this->runner->entityCompMgr.EntityByComponents(
      components::World());
}

/////////////////////////////////////////////////
void SyncManager::DistributePerformers()
{
  if (this->role == NetworkRole::SimulationPrimary) {
    auto mgr = dynamic_cast<NetworkManagerPrimary *>(
        this->runner->networkMgr.get());

    auto & secondaries = mgr->Secondaries();
    auto secondaryIt = secondaries.begin();
    auto& ecm = this->runner->entityCompMgr;

    msgs::PerformerAffinities msg;

    this->runner->entityCompMgr.Each<components::Performer,
                                     components::ParentEntity>(
      [&](const Entity & _entity,
          const components::Performer *,
          const components::ParentEntity * _parent) -> bool
      {
        auto pid = _parent->Data();
        auto parentName =
          this->runner->entityCompMgr.Component<components::Name>(pid);

        auto affinityMsg = msg.add_affinity();
        affinityMsg->set_model_name(parentName->Data());
        affinityMsg->set_entity_id(_entity);
        affinityMsg->set_secondary_prefix(secondaryIt->second->prefix);

        auto isStatic = ecm.Component<components::Static>(pid);
        *isStatic = components::Static(true);

        auto isActive = ecm.Component<components::PerformerActive>(_entity);
        *isActive = components::PerformerActive(false);

        this->runner->entityCompMgr.CreateComponent(_entity,
        components::PerformerAffinity(secondaryIt->second->prefix));

        secondaryIt++;
        if (secondaryIt == secondaries.end()) {
          secondaryIt = secondaries.begin();
        }

        return true;
      });

    for (auto & secondary : secondaries) {
      bool result;
      msgs::PerformerAffinities resp;

      std::string topic {secondary.second->prefix + "/affinity"};
      unsigned int timeout = 5000;

      bool executed = this->node.Request(topic, msg, timeout, resp, result);
      if (executed) {
        if (!result)
        {
          ignwarn << "Failed to set performer affinities for " <<
            secondary.second->prefix << " (service call failed)" <<
            std::endl;
        }
      }
      else
      {
        ignwarn << "Failed to set performer affinities for " <<
          secondary.second->prefix << " (service call timed out)" <<
          std::endl;
      }
    }
  }
  else
  {
    auto mgr = dynamic_cast<NetworkManagerSecondary *>(
        this->runner->networkMgr.get());
    auto& ecm = this->runner->entityCompMgr;
    std::string topic {mgr->Namespace() + "/affinity"};
    bool received = false;

    std::function<bool(const msgs::PerformerAffinities &,
                       msgs::PerformerAffinities &)> fcn =
      [&received, &mgr, this, &ecm](const msgs::PerformerAffinities & _req,
        msgs::PerformerAffinities &/*_resp*/) -> bool
      {
        for (int ii = 0; ii < _req.affinity_size(); ++ii) {
          const auto& affinityMsg = _req.affinity(ii);
          auto pid =
            ecm.Component<components::ParentEntity>(affinityMsg.entity_id());

          ecm.CreateComponent(affinityMsg.entity_id(),
            components::PerformerAffinity(affinityMsg.secondary_prefix()));

          auto isStatic = ecm.Component<components::Static>(pid->Data());
          auto isActive =
            ecm.Component<components::PerformerActive>(affinityMsg.entity_id());

          if (affinityMsg.secondary_prefix() == mgr->Namespace())
          {
            performers.push_back(affinityMsg.entity_id());
            *isStatic = components::Static(false);
            *isActive = components::PerformerActive(true);
          }
          else
          {
            *isStatic = components::Static(true);
            *isActive = components::PerformerActive(false);
          }
        }
        received = true;
        return true;
      };

    this->node.Advertise(topic, fcn);

    while (!received) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

/////////////////////////////////////////////////
void SyncManager::OnPose(const ignition::msgs::Pose_V & _msg)
{
  std::lock_guard<std::mutex> lock(this->poseMutex);
  this->poseMsgs.push_back(_msg);
}

/////////////////////////////////////////////////
bool SyncManager::Sync()
{
  IGN_PROFILE("SyncManager::Sync");

  // TODO(mjcarroll) this is where more advanced serialization/sync will go.
  auto& ecm = this->runner->entityCompMgr;

  if (this->role == NetworkRole::SimulationSecondary)
  {
    ignition::msgs::Pose_V msg;

    for (const auto & entity : this->performers) {
      auto pid = ecm.Component<components::ParentEntity>(entity);
      auto pose = ecm.Component<components::Pose>(pid->Data());
      auto poseMsg = msg.add_pose();
      poseMsg->set_id(entity);
      poseMsg->mutable_position()->set_x(pose->Data().Pos().X());
      poseMsg->mutable_position()->set_y(pose->Data().Pos().Y());
      poseMsg->mutable_position()->set_z(pose->Data().Pos().Z());
      poseMsg->mutable_orientation()->set_w(pose->Data().Rot().W());
      poseMsg->mutable_orientation()->set_x(pose->Data().Rot().X());
      poseMsg->mutable_orientation()->set_y(pose->Data().Rot().Y());
      poseMsg->mutable_orientation()->set_z(pose->Data().Rot().Z());
    }
    this->posePub.Publish(msg);
  }
  else
  {
    std::lock_guard<std::mutex> lock(this->poseMutex);
    for (const auto& msg : this->poseMsgs)
    {
      for (int ii = 0; ii < msg.pose_size(); ++ii)
      {
        const auto& poseMsg = msg.pose(ii);
        auto pid = ecm.Component<components::ParentEntity>(poseMsg.id());
        auto pose = ecm.Component<components::Pose>(pid->Data());

        ignition::math::Pose3d newPose;
        newPose.Set(
            ignition::math::Vector3d(
              poseMsg.position().x(),
              poseMsg.position().y(),
              poseMsg.position().z()),
            ignition::math::Quaterniond(
              poseMsg.orientation().w(),
              poseMsg.orientation().x(),
              poseMsg.orientation().y(),
              poseMsg.orientation().z()));
        *pose = components::Pose(newPose);
      }
    }
    this->poseMsgs.clear();
  }
  return true;
}
