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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/math/Helpers.hh>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include "ignition/gazebo/components/LogPlaybackStatistics.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "PlaybackScrubber.hh"

namespace ignition::gazebo
{
  class PlaybackScrubberPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief The start time of the log file
    public: common::Time startTime = common::Time::Zero;

    /// \brief The end time of the log fiel
    public: common::Time endTime = common::Time::Zero;

    /// \brief The progress as a percentage of how far we
    /// are into the log file
    public: double progress;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
PlaybackScrubber::PlaybackScrubber() : GuiSystem(),
  dataPtr(std::make_unique<PlaybackScrubberPrivate>())
{
}

/////////////////////////////////////////////////
PlaybackScrubber::~PlaybackScrubber() = default;

/////////////////////////////////////////////////
void PlaybackScrubber::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Playback Scrubber";
}

/////////////////////////////////////////////////
double PlaybackScrubber::CalculateProgress(const common::Time &_currentTime)
{
  if (this->dataPtr->startTime == this->dataPtr->endTime)
    return 0.0;

  double currentTime = _currentTime.Double();
  double startTime = this->dataPtr->startTime.Double();
  double endTime = this->dataPtr->endTime.Double();

  double numerator = currentTime - startTime;
  double denominator = endTime - startTime;
  double percentage = numerator / denominator;
  if (percentage < 0.0)
    percentage = 0;
  if (percentage > 1.0)
    percentage = 1.0;
  return percentage;
}

//////////////////////////////////////////////////
void PlaybackScrubber::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (this->dataPtr->startTime == common::Time::Zero &&
      this->dataPtr->endTime == common::Time::Zero)
  {
    _ecm.Each<components::LogPlaybackStatistics>(
      [this](const Entity &,
           const components::LogPlaybackStatistics *_logStatComp)->bool
      {
        this->dataPtr->startTime.sec =
          _logStatComp->Data().start_time().sec();
        this->dataPtr->startTime.nsec =
          _logStatComp->Data().start_time().nsec();

        this->dataPtr->endTime.sec =
          _logStatComp->Data().end_time().sec();
        this->dataPtr->endTime.nsec =
          _logStatComp->Data().end_time().nsec();

        return true;
      });
  }
  auto simTime = math::durationToSecNsec(_info.simTime);
  auto currentTime = common::Time(simTime.first, simTime.second);
  this->dataPtr->progress = CalculateProgress(currentTime);
  this->newProgress();
}

/////////////////////////////////////////////////
double PlaybackScrubber::Progress()
{
  return this->dataPtr->progress;
}

/////////////////////////////////////////////////
void PlaybackScrubber::OnDrag(double _value)
{
  const std::string topic = "/world/default/playback/control";
  unsigned int timeout = 1000;
  msgs::Boolean res;
  bool result{false};

  common::Time totalTime = this->dataPtr->endTime - this->dataPtr->startTime;
  double totalTimeDouble = totalTime.Double();
  common::Time newTime(totalTimeDouble * _value);

  msgs::LogPlaybackControl playbackMsg;

  playbackMsg.mutable_seek()->set_sec(newTime.sec);
  playbackMsg.mutable_seek()->set_nsec(newTime.nsec);
  playbackMsg.set_pause(true);
  this->dataPtr->node.Request(topic, playbackMsg, timeout, res, result);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::PlaybackScrubber,
                    ignition::gui::Plugin)
