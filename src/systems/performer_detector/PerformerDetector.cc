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

#include <gz/msgs/pose.pb.h>

#include <gz/common/Profiler.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/Box.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/Pose.hh"

#include "PerformerDetector.hh"

using namespace gz;
using namespace sim;
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
    gzerr << "PerformerDetector should be attached to a model entity. "
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
    gzerr << "'<geometry><box>' is a required parameter for "
              "PerformerDetector. Failed to initialize.\n";
    return;
  }

  if (sdfClone->HasElement("pose"))
  {
    this->poseOffset = sdfClone->Get<math::Pose3d>("pose");
  }

  // Load optional header data.
  if (sdfClone->HasElement("header_data"))
  {
    auto headerElem = sdfClone->GetElement("header_data");
    while (headerElem)
    {
      std::string key = headerElem->Get<std::string>("key", "").first;
      std::string value = headerElem->Get<std::string>("value", "").first;
      if (!key.empty() && !value.empty())
        this->extraHeaderData[key] = value;
      else if (key.empty() && !value.empty())
      {
        gzerr << "Performer detector[" << this->detectorName << "] has an "
          << "empty <key> with an associated <value> of [" << value << "]. "
          << "This <header_data> will be ignored.\n";
      }
      else if (value.empty() && !key.empty())
      {
        gzerr << "Performer detector[" << this->detectorName << "] has an "
          << "empty <value> with an associated <key> of [" << key << "]. "
          << "This <header_data> will be ignored.\n";
      }
      else
      {
        gzerr << "Performer detector[" << this->detectorName << "] has an "
          << "empty <header_data> element. This <header_data> will be "
          << "ignored\n";
      }

      headerElem = headerElem->GetNextElement("header_data");
    }
  }

  std::string defaultTopic{"/model/" + this->model.Name(_ecm) +
                             "/performer_detector/status"};
  auto topic = _sdf->Get<std::string>("topic", defaultTopic).first;

  gzmsg << "PerformerDetector publishing messages on "
         << "[" << topic << "]" << std::endl;

  transport::Node node;
  this->pub = node.Advertise<msgs::Pose>(topic);
  this->initialized = true;
}

//////////////////////////////////////////////////
void PerformerDetector::PostUpdate(
  const UpdateInfo &_info,
  const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PerformerDetector::PostUpdate");

  if (this->initialized && !this->model.Valid(_ecm))
  {
    // Deactivate this performer if the parent model has been removed, for
    // example, by the level manager
    this->initialized = false;
    return;
  }

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
          gzerr << "Internal error: geometry of performer [" << _entity
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
  {
    auto *headerData = msg.mutable_header()->add_data();
    headerData->set_key("count");
    headerData->add_value(std::to_string(this->detectedEntities.size()));
  }

  // Include the optional extra header data.
  for (const auto &data : this->extraHeaderData)
  {
    auto *headerData = msg.mutable_header()->add_data();
    headerData->set_key(data.first);
    headerData->add_value(data.second);
  }

  this->pub.Publish(msg);
}

GZ_ADD_PLUGIN(PerformerDetector,
                    System,
                    PerformerDetector::ISystemConfigure,
                    PerformerDetector::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(PerformerDetector,
                          "gz::sim::systems::PerformerDetector")
