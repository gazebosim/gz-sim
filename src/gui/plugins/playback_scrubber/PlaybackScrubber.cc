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

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;

    public: common::Time startTime = common::Time::Zero;
    
    public: common::Time endTime = common::Time::Zero;
    
    public: double progress;

    public: std::string worldName = "";
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

  // For shapes requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);

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
  return numerator / denominator;
}

//////////////////////////////////////////////////
void PlaybackScrubber::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{

  if (this->dataPtr->startTime == common::Time::Zero &&
      this->dataPtr->endTime == common::Time::Zero)
  {
    _ecm.Each<components::LogPlaybackStatistics>(
      [this](const Entity &_logStats,
           const components::LogPlaybackStatistics *_logStatComp)->bool
      {
        this->dataPtr->startTime.sec = _logStatComp->Data().start_time().sec();
        this->dataPtr->startTime.nsec = _logStatComp->Data().start_time().nsec();
        this->dataPtr->endTime.sec = _logStatComp->Data().end_time().sec();
        this->dataPtr->endTime.nsec = _logStatComp->Data().end_time().nsec();
        return true;
      });
  }
  if (this->dataPtr->worldName.empty())
  {
    // TODO(anyone) Only one scene is supported for now
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &/*_entity*/,
          const components::World * /* _world */ ,
          const components::Name *_name)->bool
        {
          this->dataPtr->worldName = _name->Data();
          return true;
        });
    ignwarn << "worldname is " << this->dataPtr->worldName << std::endl;
  }
  auto simTime = math::durationToSecNsec(_info.simTime);
  auto currentTime = common::Time(simTime.first, simTime.second);
  this->dataPtr->progress = CalculateProgress(currentTime);
  ignwarn << "progress is " << this->dataPtr->progress << std::endl;
  this->newProgress();
}

/////////////////////////////////////////////////
double PlaybackScrubber::Progress()
{
  return this->dataPtr->progress;
}

/////////////////////////////////////////////////
void PlaybackScrubber::OnDrag(double value, double from, double to)
{
  ignwarn << "value: " << value << std::endl;
  ignwarn << "from: " << from << std::endl;
  ignwarn << "to: " << to << std::endl;
  ignwarn << "percentage: " << ((value - from)/ (to - from)) << std::endl;
  double percentageScrolled = (value - from)/ (to - from);
  common::Time totalTime = this->dataPtr->endTime - this->dataPtr->startTime;
  double totalTimeDouble = totalTime.Double();
  ignwarn << "total time as double is " << totalTimeDouble << std::endl;
  ignwarn << "scrolled to time as double is " << totalTimeDouble * percentageScrolled << std::endl;
  common::Time newTime(totalTimeDouble * value);
  // TODO make transport request on topic /world/shapes/playback/control
  // with message type LogPlaybackControl, possibly find a way only to do
  // it when mouse is released
  const std::string topic = "/world/default/playback/control";
  msgs::LogPlaybackControl playbackMsg;
  playbackMsg.mutable_seek()->set_sec(newTime.sec);
  playbackMsg.mutable_seek()->set_nsec(newTime.nsec);
  playbackMsg.set_pause(true);
  ignwarn << "playback seconds " << playbackMsg.seek().sec() << std::endl;
  ignwarn << "playback nanoseconds " << playbackMsg.seek().nsec() << std::endl;
  unsigned int timeout = 1000;
  msgs::Boolean res;
  bool result{false};
  this->dataPtr->node.Request(topic, playbackMsg, timeout, res, result);
  ignwarn << "result is " << result << std::endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::PlaybackScrubber,
                    ignition::gui::Plugin)
