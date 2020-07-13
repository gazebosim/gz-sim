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

#include <ignition/common/Profiler.hh>
#include <ignition/math/AxisAlignedBox.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "PerformerDetector.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void PerformerDetector::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  this->model = Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    ignerr << "PerformerDetector should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->detectorName = this->model.Name(_ecm);

  auto sdfClone = _sdf->Clone();
  bool hasGeometry{false};
  if (sdfClone->HasElement("geometry"))
  {
    auto geom = sdfClone->GetElement("geometry");
    if (geom->HasElement("box"))
    {
      auto box = geom->GetElement("box");
      auto boxSize = box->Get<math::Vector3d>("size");
      this->detectorGeometry = math::AxisAlignedBox(-boxSize / 2, boxSize / 2);
      hasGeometry = true;
    }
  }

  if (!hasGeometry)
  {
    ignerr << "'<geometry><box>' is a required parameter for "
              "PerformerDetector. Failed to initialize.\n";
    return;
  }

  if (sdfClone->HasElement("pose"))
  {
    this->poseOffset = sdfClone->Get<math::Pose3d>("pose");
  }

  std::string defaultTopic{"/model/" + this->model.Name(_ecm) +
                             "/performer_detector/status"};
  auto topic = _sdf->Get<std::string>("topic", defaultTopic).first;

  ignmsg << "PerformerDetector publishing messages on "
         << "[" << topic << "]" << std::endl;

  transport::Node node;
  this->pub = node.Advertise<msgs::Pose>(topic);
  this->initialized = true;
}

//////////////////////////////////////////////////
void PerformerDetector::PostUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  const ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("PerformerDetector::PostUpdate");

  if (_info.paused)
    return;

  if (!this->initialized)
  {
    return;
  }

  auto modelPose =
      _ecm.Component<components::Pose>(this->model.Entity())->Data();

  // Double negative because AxisAlignedBox does not currently have operator+
  // that takes a position
  auto region = this->detectorGeometry -
    (-(modelPose.Pos() + modelPose.Rot() * this->poseOffset.Pos()));

  _ecm.Each<components::Performer, components::Geometry,
            components::ParentEntity>(
      [&](const Entity &_entity, const components::Performer *,
          const components::Geometry *_geometry,
          const components::ParentEntity *_parent) -> bool
      {
        auto pose = _ecm.Component<components::Pose>(_parent->Data())->Data();
        auto name = _ecm.Component<components::Name>(_parent->Data())->Data();
        const math::Pose3d relPose = modelPose.Inverse() * pose;

        // We assume the geometry contains a box.
        auto perfBox = _geometry->Data().BoxShape();
        if (nullptr == perfBox)
        {
          ignerr << "Internal error: geometry of performer [" << _entity
                 << "] missing box." << std::endl;
          return true;
        }

        math::AxisAlignedBox performerVolume{pose.Pos() - perfBox->Size() / 2,
                                             pose.Pos() + perfBox->Size() / 2};

        bool alreadyDetected = this->IsAlreadyDetected(_entity);
        if (region.Intersects(performerVolume))
        {
          if (!alreadyDetected)
          {
            this->AddToDetected(_entity);
            this->Publish(_entity, name, true, relPose, _info.simTime);
          }
        }
        else if (alreadyDetected)
        {
          this->RemoveFromDetected(_entity);
          this->Publish(_entity, name, false, relPose, _info.simTime);
        }

        return true;
      });
}

//////////////////////////////////////////////////
bool PerformerDetector::IsAlreadyDetected(const Entity &_entity) const
{
  return this->detectedEntities.find(_entity) != this->detectedEntities.end();
}

//////////////////////////////////////////////////
void PerformerDetector::AddToDetected(const Entity &_entity)
{
  this->detectedEntities.insert(_entity);
}

//////////////////////////////////////////////////
void PerformerDetector::RemoveFromDetected(const Entity &_entity)
{
  this->detectedEntities.erase(_entity);
}

//////////////////////////////////////////////////
void PerformerDetector::Publish(
    const Entity &_entity, const std::string &_name, bool _state,
    const math::Pose3d &_pose,
    const std::chrono::steady_clock::duration &_stamp)
{
  msgs::Pose msg = msgs::Convert(_pose);
  msg.set_name(_name);
  msg.set_id(_entity);

  auto stamp = math::durationToSecNsec(_stamp);
  msg.mutable_header()->mutable_stamp()->set_sec(stamp.first);
  msg.mutable_header()->mutable_stamp()->set_nsec(stamp.second);

  {
    auto *headerData = msg.mutable_header()->add_data();
    headerData->set_key("frame_id");
    headerData->add_value(this->detectorName);
  }
  {
    auto *headerData = msg.mutable_header()->add_data();
    headerData->set_key("state");
    headerData->add_value(std::to_string(_state));
  }

  this->pub.Publish(msg);
}

IGNITION_ADD_PLUGIN(PerformerDetector,
                    ignition::gazebo::System,
                    PerformerDetector::ISystemConfigure,
                    PerformerDetector::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(PerformerDetector,
                          "ignition::gazebo::systems::PerformerDetector")
