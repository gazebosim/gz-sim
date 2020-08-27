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

#ifndef IGNITION_GAZEBO_GUI_AXESCONFIG_HH_
#define IGNITION_GAZEBO_GUI_AXESCONFIG_HH_

#include <memory>
#include <string>

#include <ignition/gui/Plugin.hh>
#include <ignition/rendering.hh>

namespace ignition
{
namespace gazebo
{
  class AxesConfigPrivate;

  class AxesConfig : public ignition::gui::Plugin
  {
    Q_OBJECT

    /// \internal
    /// \brief QProperty to udpate the QCombobox with the entities
    Q_PROPERTY(QStringList comboList
               READ comboList
               WRITE SetComboList
               NOTIFY ComboListChanged)

    /// \brief Constructor
    public: AxesConfig();

    /// \brief Destructor
    public: ~AxesConfig() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *) override;

    // Documentation inherited
    protected: bool eventFilter(QObject *_obj, QEvent *_event) override;

    /// \brief Update arrows properties (visibility, pose, type, ...)
    public: void UpdateOriginArrows();

    /// \brief Loaded the AxesVisual based on the name
    public: void LoadAxesbyName(const std::string & name);

    /// \brief update the active axes
    /// We need to do that in other to render the axes when moving
    /// a object in the scene
    private: void UpdateActiveAxes();

    /// \brief function to find all the entities in the scene based
    /// on the visual names
    private: void EntitiesInScene();

    /// \brief set a new combobox list and emit the signal ComboListChanged
    public: void SetComboList(const QStringList &comboList);

    /// \brief Callback to update the size of the arrows
    /// \param[in] _size size of the arrows in meters
    public slots: void UpdateLength(double _length);

    /// \brief Callback when checkbox is clicked.
    /// \param[in] _checked indicates show or hide arrows
    public slots: void OnShow(bool _checked);

    /// \brief Callback when combbobox is changed.
    /// \param[in] _index
    public slots: void onCurrentIndexChanged(int _index);

    /// \brief Callback when checkbox is clicked.
    /// \param[in] _checked indicates show or hide arrows
    public slots: void OnTypeAxes(bool _checked);

    /// \brief Callback to update axes pose
    /// \param[in] _x, _y, _z cartesion coordinates
    /// \param[in] _roll, _pitch, _yaw principal coordinates
    public slots: void SetPose(double _x, double _y, double _z,
                               double _roll, double _pitch, double _yaw);

    /// \brief signal to emit when a new list is ready to show in the combobox
    signals: void ComboListChanged();

    /// \brief Qt String list with the item of the combobox.
    public: QStringList itemComboList;

    /// \internal
    /// \brief Return a Qt String list with the item of the combobox.
    private: const QStringList comboList();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<AxesConfigPrivate> dataPtr;

    /// \internal
    /// \brief QProperty to udpate the length of each axes on the GUI
    Q_PROPERTY(double length READ length)

    /// \internal
    /// \brief QProperty to udpate the X position of each axes on the GUI
    Q_PROPERTY(double axesX READ axesX)

    /// \internal
    /// \brief QProperty to udpate the Y position of each axes on the GUI
    Q_PROPERTY(double axesY READ axesY)

    /// \internal
    /// \brief QProperty to udpate the Z position of each axes on the GUI
    Q_PROPERTY(double axesZ READ axesZ)

    /// \internal
    /// \brief QProperty to udpate the roll orientation of each axes on the GUI
    Q_PROPERTY(double axesRoll READ axesRoll)

    /// \internal
    /// \brief QProperty to udpate the pitch orientation of each axes on the GUI
    Q_PROPERTY(double axesPitch READ axesPitch)

    /// \internal
    /// \brief QProperty to udpate the yaw orientation of each axes on the GUI
    Q_PROPERTY(double axesYaw READ axesYaw)

    /// \internal
    /// \brief return length of the active axis
    private: double length() const;

    /// \internal
    /// \brief return X position of the active axis
    private: double axesX() const;

    /// \internal
    /// \brief return Y position of the active axis
    private: double axesY() const;

    /// \internal
    /// \brief return Z position of the active axis
    private: double axesZ() const;

    /// \internal
    /// \brief return roll of the active axis
    private: double axesRoll() const;

    /// \internal
    /// \brief return pitch of the active axis
    private: double axesPitch() const;

    /// \internal
    /// \brief return yaw of the active axis
    private: double axesYaw() const;
  };
}
}

#endif
