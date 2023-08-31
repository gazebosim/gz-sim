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

#include "PlaybackScrubber.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/log_playback_control.pb.h>

#include <chrono>
#include <ctime>
#include <iostream>
#include <regex>
#include <string>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/LogPlaybackStatistics.hh"

namespace gz::sim
{
  class PlaybackScrubberPrivate
  {
    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief The start time of the log file
    public: std::chrono::steady_clock::time_point startTime;

    /// \brief The end time of the log file
    public: std::chrono::steady_clock::time_point endTime;

    /// \brief The current time of the log file
    public: std::chrono::steady_clock::time_point currentTime;

    /// \brief The name of the world
    public: std::string worldName = "";

    /// \brief The progress as a percentage of how far we
    /// are into the log file
    public: double progress{0.0};

    /// \brief Bool holding if the simulation is currently paused.
    public: bool paused{false};
  };
}

using namespace gz;
using namespace sim;

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
double PlaybackScrubber::CalculateProgress()
{
  if (this->dataPtr->startTime == this->dataPtr->endTime)
    return 0.0;

  auto startTime = this->dataPtr->startTime;
  auto endTime = this->dataPtr->endTime;
  auto currentTime = this->dataPtr->currentTime;

  auto totalDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      endTime - startTime).count();
  auto currentDuration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      currentTime - startTime).count();

  auto percentage =
    static_cast<double>(currentDuration) / static_cast<double>(totalDuration);

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
  auto startTime = this->dataPtr->startTime;
  auto endTime = this->dataPtr->endTime;
  auto totalDuration =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
        endTime - startTime).count();

  // Set the start and end times
  if (totalDuration <= 0)
  {
    _ecm.Each<components::LogPlaybackStatistics>(
      [this](const Entity &,
           const components::LogPlaybackStatistics *_logStatComp)->bool
      {
        auto startSeconds =
          _logStatComp->Data().start_time().sec();
        auto startNanoseconds =
          _logStatComp->Data().start_time().nsec();
        auto endSeconds =
          _logStatComp->Data().end_time().sec();
        auto endNanoseconds =
          _logStatComp->Data().end_time().nsec();
        this->dataPtr->startTime =
          math::secNsecToTimePoint(startSeconds, startNanoseconds);
        this->dataPtr->endTime =
          math::secNsecToTimePoint(endSeconds, endNanoseconds);
        return true;
      });
  }

  // Populate the world name
  if (this->dataPtr->worldName == "")
  {
    // TODO(anyone) Only one world is supported for now
    auto worldNames = gz::gui::worldNames();
    if (worldNames.size() >= 1)
    {
      this->dataPtr->worldName = worldNames[0].toStdString();
    }
  }

  auto simTime = math::durationToSecNsec(_info.simTime);
  this->dataPtr->currentTime =
    math::secNsecToTimePoint(simTime.first, simTime.second);
  this->dataPtr->progress = CalculateProgress();
  this->dataPtr->paused = _info.paused;
  this->newProgress();
}

/////////////////////////////////////////////////
double PlaybackScrubber::Progress()
{
  return this->dataPtr->progress;
}

/////////////////////////////////////////////////
QString PlaybackScrubber::StartTimeAsString()
{
  return QString::fromStdString(
      math::timePointToString(this->dataPtr->startTime));
}

/////////////////////////////////////////////////
QString PlaybackScrubber::EndTimeAsString()
{
  return QString::fromStdString(
      math::timePointToString(this->dataPtr->endTime));
}

/////////////////////////////////////////////////
QString PlaybackScrubber::CurrentTimeAsString()
{
  return QString::fromStdString(
      math::timePointToString(this->dataPtr->currentTime));
}

/////////////////////////////////////////////////
void PlaybackScrubber::OnTimeEntered(const QString &_time)
{
  std::string time = _time.toStdString();
  std::chrono::steady_clock::time_point enteredTime =
    math::stringToTimePoint(time);
  if (enteredTime == math::secNsecToTimePoint(-1, 0))
  {
    gzwarn << "Invalid time entered. "
      "The format is dd hh:mm:ss.nnn" << std::endl;
    return;
  }

  // Check for time out of bounds
  if (enteredTime < this->dataPtr->startTime)
    enteredTime = this->dataPtr->startTime;
  else if (enteredTime > this->dataPtr->endTime)
    enteredTime = this->dataPtr->endTime;

  auto pairTime = math::timePointToSecNsec(enteredTime);

  unsigned int timeout = 1000;
  msgs::Boolean res;
  bool result{false};
  msgs::LogPlaybackControl playbackMsg;

  // Set time and make request
  playbackMsg.mutable_seek()->set_sec(pairTime.first);
  playbackMsg.mutable_seek()->set_nsec(pairTime.second);
  playbackMsg.set_pause(true);
  this->dataPtr->node.Request(
      "/world/" + this->dataPtr->worldName + "/playback/control",
      playbackMsg, timeout, res, result);
}

/////////////////////////////////////////////////
void PlaybackScrubber::OnDrop(double _value)
{
  unsigned int timeout = 1000;
  msgs::Boolean res;
  bool result{false};

  auto totalDuration =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
        this->dataPtr->endTime - this->dataPtr->startTime);
  auto newTime = this->dataPtr->startTime +
    std::chrono::duration_cast<std::chrono::nanoseconds>(
        totalDuration * _value);
  std::pair<int64_t, int64_t> jumpToTime = math::timePointToSecNsec(newTime);

  msgs::LogPlaybackControl playbackMsg;

  playbackMsg.mutable_seek()->set_sec(jumpToTime.first);
  playbackMsg.mutable_seek()->set_nsec(jumpToTime.second);
  playbackMsg.set_pause(this->dataPtr->paused);
  this->dataPtr->node.Request(
      "/world/" + this->dataPtr->worldName + "/playback/control",
      playbackMsg, timeout, res, result);
}

// Register this plugin
GZ_ADD_PLUGIN(PlaybackScrubber,
              gz::gui::Plugin)
