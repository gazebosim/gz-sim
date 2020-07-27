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

#ifndef IGNITION_GAZEBO_GUI_VISUALIZECONTACTS_HH_
#define IGNITION_GAZEBO_GUI_VISUALIZECONTACTS_HH_

#include <memory>

#include <ignition/gazebo/gui/GuiSystem.hh>

#include "ignition/gui/qt.h"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
{
  class VisualizeContactsPrivate;

  /// \brief Visualize the contacts returned by the Physics plugin. Use the
  /// checkbox to turn visualization on or off and spin boxes to change
  /// the size of the markers.
  class VisualizeContacts : public ignition::gazebo::GuiSystem
  {
    Q_OBJECT

    /// \brief Constructor
    public: VisualizeContacts();

    /// \brief Destructor
    public: ~VisualizeContacts() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
        EntityComponentManager &_ecm) override;

    /// \brief Callback when checkbox state is changed
    /// \param[in] _checked indicates show or hide contacts
    public slots: void OnVisualize(bool _checked);

    /// \brief Update the radius of the contact
    /// \param[in] _radius new radius of the contact
    public slots: void UpdateRadius(double _radius);

    /// \brief Update the scale of the contact. By scale we mean how many
    /// meters per newton the marker will be long
    /// \param[in] _scale new scale of the contact
    public slots: void UpdateScale(double _scale);

    /// \brief Update the update period of the markers
    /// \param[in] _period new update period
    public slots: void UpdatePeriod(double _period);

    /// \internal
    /// \brief Pointer to private data
    private: std::unique_ptr<VisualizeContactsPrivate> dataPtr;
  };
}
}
}

#endif
