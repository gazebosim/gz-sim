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

#ifndef IGNITION_GAZEBO_GUI_COPYPASTE_HH_
#define IGNITION_GAZEBO_GUI_COPYPASTE_HH_

#include <memory>

#include <ignition/gui/Plugin.hh>

namespace ignition
{
namespace gazebo
{
  class CopyPastePrivate;

  class CopyPaste : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: CopyPaste();

    /// \brief Destructor
    public: ~CopyPaste() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    /// \brief Callback to copy the selected entity
    /// \param[in] _copyItemName The name of the entity to be copied
    public slots: void OnCopy(const QString &_copyItemName);

    /// \brief Callback to paste the data that has been copied, if copied data
    /// exists.
    public slots: void OnPaste();

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<CopyPastePrivate> dataPtr;
  };
}
}

#endif
