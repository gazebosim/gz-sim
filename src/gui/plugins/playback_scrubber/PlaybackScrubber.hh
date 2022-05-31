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

#ifndef GZ_SIM_GUI_PLAYBACK_SCRUBBER_HH_
#define GZ_SIM_GUI_PLAYBACK_SCRUBBER_HH_

#include <chrono>
#include <memory>

#include <gz/sim/gui/GuiSystem.hh>
#include <gz/gui/Plugin.hh>

namespace gz
{
namespace sim
{
  class PlaybackScrubberPrivate;

  /// \brief Provides slider and functionality for log playback.
  /// to the scene
  class PlaybackScrubber : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: PlaybackScrubber();

    /// \brief Destructor
    public: ~PlaybackScrubber() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &, EntityComponentManager &) override;

    /// \brief Calculate the percentage that `_currentTime`, eg, halfway
    /// through the log would evaluate to 0.50
    /// \return The progress as a percentage of how far the log playback
    /// has advanced
    public: double CalculateProgress();

    /// \brief Get the current progress as a percentage of how far the log
    /// playback has advanced
    /// \return The progress as a value from 0 to 1, inclusive
    public slots: double Progress();

    /// \brief Get the starting time of the log playback.
    /// \return The start time in format dd hh:mm:ss.nnn as a QString.
    public slots: QString StartTimeAsString();

    /// \brief Get the end time of the log playback.
    /// \return The end time in format dd hh:mm:ss.nnn as a QString.
    public slots: QString EndTimeAsString();

    /// \brief Get the current time of the log playback.
    /// \return The current time in format dd hh:mm:ss.nnn as a QString.
    public slots: QString CurrentTimeAsString();

    /// \brief Callback in Qt thread when the slider is released.
    /// \param[in] _value The current value of the slider, from 0 to 1,
    /// inclusive
    public slots: void OnDrop(double _value);

    /// \brief Callback when a time is entered by the user.
    /// \param[in] _time The time in format dd hh:mm:ss.nnn
    public slots: void OnTimeEntered(const QString &_time);

    /// \brief Notify that progress has advanced in the log file.
    signals: void newProgress();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<PlaybackScrubberPrivate> dataPtr;
  };
}
}

#endif
