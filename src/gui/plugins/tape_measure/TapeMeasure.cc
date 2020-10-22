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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Grid.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "TapeMeasure.hh"

namespace ignition::gazebo
{
  class TapeMeasurePrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    public: bool measure = false;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TapeMeasure::TapeMeasure()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<TapeMeasurePrivate>())
{
}

/////////////////////////////////////////////////
TapeMeasure::~TapeMeasure() = default;

/////////////////////////////////////////////////
void TapeMeasure::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Tape measure";

  ignition::gui::App()->findChild<ignition::gui::MainWindow *>
      ()->installEventFilter(this);
}

/////////////////////////////////////////////////
void TapeMeasure::OnMeasure()
{
  this->dataPtr->measure = true;
}

/////////////////////////////////////////////////
bool TapeMeasure::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    // TODO do any rendering here
    if (this->dataPtr->measure)
    {
      ignwarn << "Measuring" << std::endl;
      this->dataPtr->measure = false;
    }
  }
  return QObject::eventFilter(_obj, _event);
}
// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::TapeMeasure,
                    ignition::gui::Plugin)
