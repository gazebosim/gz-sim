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

#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/Utility.hh>

#include "Pose3d.hh"

using namespace gz;
using namespace sim;
using namespace inspector;

/////////////////////////////////////////////////
Pose3d::Pose3d(ComponentInspector *_inspector)
{
  _inspector->Context()->setContextProperty("Pose3dImpl", this);
  this->inspector = _inspector;

  this->inspector->AddUpdateViewCb(components::Pose::typeId,
      std::bind(&Pose3d::UpdateView<components::Pose>, this,
      std::placeholders::_1, std::placeholders::_2));
  this->inspector->AddUpdateViewCb(components::WorldPose::typeId,
      std::bind(&Pose3d::UpdateView<components::WorldPose>, this,
      std::placeholders::_1, std::placeholders::_2));
  this->inspector->AddUpdateViewCb(components::WorldPoseCmd::typeId,
      std::bind(&Pose3d::UpdateView<components::WorldPoseCmd>, this,
      std::placeholders::_1, std::placeholders::_2));
}

/////////////////////////////////////////////////
void Pose3d::OnPose(double _x, double _y, double _z, double _roll,
    double _pitch, double _yaw)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &, const bool _result)
  {
    if (!_result)
        gzerr << "Error setting pose" << std::endl;
  };

  msgs::Pose req;
  req.set_id(this->inspector->GetEntity());
  msgs::Set(req.mutable_position(), math::Vector3d(_x, _y, _z));
  msgs::Set(req.mutable_orientation(), math::Quaterniond(_roll, _pitch, _yaw));
  std::string poseCmdService("/world/" + this->inspector->WorldName()
      + "/set_pose");
  this->inspector->TransportNode().Request(poseCmdService, req, cb);
}
