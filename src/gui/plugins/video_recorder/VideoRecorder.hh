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

#ifndef GZ_SIM_GUI_VIDEORECORDER_HH_
#define GZ_SIM_GUI_VIDEORECORDER_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz
{
namespace sim
{
  class VideoRecorderPrivate;

  /// \brief Provides video recording cababilities to the 3D scene.
  class VideoRecorder : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: VideoRecorder();

    /// \brief Destructor
    public: ~VideoRecorder() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info, EntityComponentManager &_ecm)
      override;

    // Documentation inherited
    public: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Callback when video record start request is received
    /// \param[in] _format Video encoding format
    public slots: void OnStart(const QString &_format);

    /// \brief Callback when video record stop request is received.
    public slots: void OnStop();

    /// \brief Callback when user selects a path to save the recorded video
    /// \param[in] _url Path of the file to save the recorded video
    public slots: void OnSave(const QString &_url);

    /// \brief Callback when user cancels saving the recorded video
    public slots: void OnCancel();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<VideoRecorderPrivate> dataPtr;
  };
}
}

#endif
