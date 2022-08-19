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

#include "ViewAngle.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/gui_camera.pb.h>
#include <ignition/msgs/vector3d.pb.h>

#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace gz::sim
{
  class ViewAnglePrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect angle mode
    public: std::mutex mutex;

    /// \brief View Angle service name
    public: std::string viewAngleService;

    /// \brief Move gui camera to pose service name
    public: std::string moveToPoseService;

    /// \brief gui camera pose
    public: math::Pose3d camPose;
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
ViewAngle::ViewAngle()
  : gui::Plugin(), dataPtr(std::make_unique<ViewAnglePrivate>())
{
}

/////////////////////////////////////////////////
ViewAngle::~ViewAngle() = default;

/////////////////////////////////////////////////
void ViewAngle::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "View Angle";

  // For view angle requests
  this->dataPtr->viewAngleService = "/gui/view_angle";

  // Subscribe to camera pose
  std::string topic = "/gui/camera/pose";
  this->dataPtr->node.Subscribe(
    topic, &ViewAngle::CamPoseCb, this);

  // Move to pose service
  this->dataPtr->moveToPoseService = "/gui/move_to/pose";
}

/////////////////////////////////////////////////
void ViewAngle::OnAngleMode(int _x, int _y, int _z)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting view angle mode" << std::endl;
  };

  msgs::Vector3d req;
  req.set_x(_x);
  req.set_y(_y);
  req.set_z(_z);

  this->dataPtr->node.Request(this->dataPtr->viewAngleService, req, cb);
}

/////////////////////////////////////////////////
QList<double> ViewAngle::CamPose() const
{
  return QList({
    this->dataPtr->camPose.Pos().X(),
    this->dataPtr->camPose.Pos().Y(),
    this->dataPtr->camPose.Pos().Z(),
    this->dataPtr->camPose.Rot().Roll(),
    this->dataPtr->camPose.Rot().Pitch(),
    this->dataPtr->camPose.Rot().Yaw()
  });
}

/////////////////////////////////////////////////
void ViewAngle::SetCamPose(double _x, double _y, double _z,
                           double _roll, double _pitch, double _yaw)
{
  this->dataPtr->camPose.Set(_x, _y, _z, _roll, _pitch, _yaw);

  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending move camera to pose request" << std::endl;
  };

  msgs::GUICamera req;
  msgs::Set(req.mutable_pose(), this->dataPtr->camPose);

  this->dataPtr->node.Request(this->dataPtr->moveToPoseService, req, cb);
}

/////////////////////////////////////////////////
void ViewAngle::CamPoseCb(const msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  math::Pose3d pose = msgs::Convert(_msg);

  if (pose != this->dataPtr->camPose)
  {
    this->dataPtr->camPose = pose;
    this->CamPoseChanged();
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ViewAngle,
                    gz::gui::Plugin)
