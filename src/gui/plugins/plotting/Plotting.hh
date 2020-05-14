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
#ifndef IGNITION_GUI_PLUGINS_PLOTTING_HH_
#define IGNITION_GUI_PLUGINS_PLOTTING_HH_

#include <ignition/gui/Application.hh>
#include <ignition/gui/qt.h>
#include <ignition/gazebo/EntityComponentManager.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include <ignition/gazebo/gui/GuiSystem.hh>

#include "PlottingInterface.hh"

#define TIMEOUT 100

namespace ignition {

namespace gazebo {

class PlottingPrivate;


class Plotting : public ignition::gazebo::GuiSystem
{
    Q_OBJECT
    /// \brief Constructor
    public: Plotting();
    /// \brief Destructor
    public: ~Plotting();

    /// \brief data_ptr holds Abstraction data of PlottingPrivate
    private: std::unique_ptr<PlottingPrivate> data_ptr;
    /// \brief get called every simulation iteration to access entities and thier components
    public: void Update(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief x position of the car
    public: float pose;
    /// \brief plotting real time
    public: float time;
    /// \brief send data to Qml to plot
    public: void UpdateGui();
};
}
}

#endif
