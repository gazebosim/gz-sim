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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/video_record.pb.h>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "VideoRecorder.hh"

namespace ignition::gazebo
{
  class VideoRecorderPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Video record service name
    public: std::string service;

    /// \brief Filename of the recorded video
    public: std::string filename;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
VideoRecorder::VideoRecorder()
  : gui::Plugin(), dataPtr(std::make_unique<VideoRecorderPrivate>())
{
}

/////////////////////////////////////////////////
VideoRecorder::~VideoRecorder() = default;

/////////////////////////////////////////////////
void VideoRecorder::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Video recorder";

  // For video record requests
  this->dataPtr->service = "/gui/record_video";
}

/////////////////////////////////////////////////
void VideoRecorder::OnStart(const QString &_format)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending video record start request" << std::endl;
  };

  std::string format = _format.toStdString();
  this->dataPtr->filename = "ign_recording." + format;

  ignition::msgs::VideoRecord req;
  req.set_start(true);
  req.set_format(format);
  req.set_save_filename(this->dataPtr->filename);
  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

/////////////////////////////////////////////////
void VideoRecorder::OnStop()
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending video record stop request" << std::endl;
  };

  ignition::msgs::VideoRecord req;
  req.set_stop(true);
  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

/////////////////////////////////////////////////
void VideoRecorder::OnSave(const QString &_url)
{
  std::string path = QUrl(_url).toLocalFile().toStdString();
  bool result = common::moveFile(this->dataPtr->filename, path);

  if (!result)
  {
    ignerr  << "Unable to rename file from[" << this->dataPtr->filename
      << "] to [" << path << "]" << std::endl;
  }
  else
  {
    ignmsg << "Video saved to: " << path << std::endl;
  }
}

/////////////////////////////////////////////////
void VideoRecorder::OnCancel()
{
  if (common::exists(this->dataPtr->filename))
    std::remove(this->dataPtr->filename.c_str());
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::VideoRecorder,
                    ignition::gui::Plugin)
