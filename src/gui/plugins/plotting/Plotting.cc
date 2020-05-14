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
#include <ignition/plugin/Register.hh>

#include "Plotting.hh"

namespace ignition::gazebo
{
    class PlottingPrivate
    {
      /// \brief Transport Object for Subscribing and Publishing Topics with Msgs
      public: Transport *transport;
      /// \brief Interface to communicate with Qml
      public: PlottingInterface *plottingIface;
    };
}

using namespace ignition;
using namespace gazebo;

Plotting ::Plotting ()  : GuiSystem() , data_ptr(new PlottingPrivate)
{
    this->pose = 0;
    this->time = 0;
    this->data_ptr->transport = new Transport();
    this->data_ptr->plottingIface = new PlottingInterface(this->data_ptr->transport);

    ignition::gui::App()->Engine()->rootContext()->setContextProperty(
        "TopicsModel", this->data_ptr->transport->GetModel());

    ignition::gui::App()->Engine()->rootContext()->setContextProperty(
                "PlottingIface", this->data_ptr->plottingIface);
}

Plotting ::~Plotting()
{
    delete this->data_ptr->plottingIface;
    delete this->data_ptr->transport;
}

void Plotting ::Update(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    int realTime = _info.realTime.count()/1000000000;

    // get the blue car entity by its name
    auto CarEntity = _ecm.EntityByComponents(components::Name("vehicle_blue"));
    if (!CarEntity)
        return;

//    // get the position component of the car entity
    auto _pose = _ecm.Component<components::Pose>(CarEntity);
    if (!_pose)
        return;

    this->pose = _pose->Data().Pos().X();

    if (realTime  <= this->time )
        return;

    this->time = realTime;
    this->UpdateGui();
}


void Plotting ::UpdateGui()
{
    this->data_ptr->plottingIface->emitPlotting(0, this->time,this->pose);
    this->data_ptr->plottingIface->emitPlotting(1, this->time,this->data_ptr->transport->GetValue());

    // cout << "time:" << this->time << "  " << this->data_ptr->transport->GetValue().toFloat() << endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Plotting ,
                    ignition::gazebo::GuiSystem,
                    ignition::gui::Plugin)
