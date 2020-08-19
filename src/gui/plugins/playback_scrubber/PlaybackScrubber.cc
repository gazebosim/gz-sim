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

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include "ignition/gazebo/components/LogPlaybackStatistics.hh"
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

//////////////////////////////////////////////////
void PlaybackScrubber::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  _ecm.Each<components::LogPlaybackStatistics>(
    [&](const Entity &_logStats,
         const components::LogPlaybackStatistics *_logStatComp)->bool
    {
      ignwarn << "start time s " << _logStatComp->Data().start_time().sec() << std::endl;
      ignwarn << "start time ns " << _logStatComp->Data().start_time().nsec() << std::endl;
      ignwarn << "end time s " << _logStatComp->Data().end_time().sec() << std::endl;
      ignwarn << "end time ns " << _logStatComp->Data().end_time().nsec() << std::endl;
      return true;
    });
}

/////////////////////////////////////////////////
void PlaybackScrubber::OnDrag(double value, double from, double to)
{
  ignwarn << "value: " << value << std::endl;
  ignwarn << "from: " << from << std::endl;
  ignwarn << "to: " << to << std::endl;
  ignwarn << "percentage: " << ((value - from)/ (to - from)) << std::endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::PlaybackScrubber,
                    ignition::gui::Plugin)
