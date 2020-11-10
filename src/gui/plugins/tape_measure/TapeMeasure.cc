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
#include <ignition/msgs/marker.pb.h>

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

    public: int startPointId = 1;

    public: int endPointId = 2;

    public: ignition::math::Vector3d startPoint = ignition::math::Vector3d::Zero;
    public: ignition::math::Vector3d endPoint = ignition::math::Vector3d::Zero;

    public: ignition::math::Vector4d hoverColor = ignition::math::Vector4d(0.2, 0.2, 0.2, 0.5);
    public: ignition::math::Vector4d drawColor = ignition::math::Vector4d(0.2, 0.2, 0.2, 1.0);

    public: std::unordered_set<int> placedMarkers;

    public: int lineId = 3;

    public: int currentId = startPointId;

    public: double distance = 0.0;

    public: bool placed = false;
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
  this->Reset();
  this->dataPtr->measure = true;
}

/////////////////////////////////////////////////
void TapeMeasure::OnReset()
{
  this->Reset();
}

void TapeMeasure::Reset()
{
  this->dataPtr->currentId = this->dataPtr->startPointId;
  this->dataPtr->startPoint = ignition::math::Vector3d::Zero;
  this->dataPtr->endPoint = ignition::math::Vector3d::Zero;
  this->dataPtr->placedMarkers.clear();
  this->dataPtr->distance = 0.0;
  this->newDistance();

  if (this->dataPtr->placed)
  {
    // Delete the previously created marker
    ignition::msgs::Marker markerMsg;
    markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
    this->dataPtr->node.Request("/marker", markerMsg);
    this->dataPtr->placed = false;
  }
}

double TapeMeasure::Distance()
{
  return this->dataPtr->distance;
}

void TapeMeasure::DrawPoint(int id, math::Vector3d &_point, math::Vector4d &_color)
{
  ignition::msgs::Marker markerMsg;
  if (this->dataPtr->placedMarkers.find(id) != this->dataPtr->placedMarkers.end())
  {
    // Delete the previously created marker
    markerMsg.set_ns("default");
    markerMsg.set_id(id);
    markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
    this->dataPtr->node.Request("/marker", markerMsg);
  }
  else
  {
    this->dataPtr->placedMarkers.insert(id);
  }

  markerMsg.set_ns("default");
  markerMsg.set_id(id);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::SPHERE);
  ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.1, 0.1, 0.1));
  markerMsg.mutable_material()->mutable_ambient()->set_r(_color[0]);
  markerMsg.mutable_material()->mutable_ambient()->set_g(_color[1]);
  markerMsg.mutable_material()->mutable_ambient()->set_b(_color[2]);
  markerMsg.mutable_material()->mutable_ambient()->set_a(_color[3]);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(_color[0]);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(_color[1]);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(_color[2]);
  markerMsg.mutable_material()->mutable_diffuse()->set_a(_color[3]);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(_point.X(), _point.Y(), _point.Z(), 0, 0, 0));
  this->dataPtr->node.Request("/marker", markerMsg);
}

void TapeMeasure::DrawLine(int id, math::Vector3d &_startPoint, math::Vector3d &_endPoint, math::Vector4d &_color)
{
  ignition::msgs::Marker markerMsg;
  if (this->dataPtr->placedMarkers.find(id) != this->dataPtr->placedMarkers.end())
  {
    // Delete the previously created marker
    markerMsg.set_ns("default");
    markerMsg.set_id(id);
    markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
    this->dataPtr->node.Request("/marker", markerMsg);
  }
  else
  {
    this->dataPtr->placedMarkers.insert(id);
  }

  markerMsg.set_ns("default");
  markerMsg.set_id(id);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  markerMsg.mutable_material()->mutable_ambient()->set_r(_color[0]);
  markerMsg.mutable_material()->mutable_ambient()->set_g(_color[1]);
  markerMsg.mutable_material()->mutable_ambient()->set_b(_color[2]);
  markerMsg.mutable_material()->mutable_ambient()->set_a(_color[3]);
  markerMsg.mutable_material()->mutable_diffuse()->set_r(_color[0]);
  markerMsg.mutable_material()->mutable_diffuse()->set_g(_color[1]);
  markerMsg.mutable_material()->mutable_diffuse()->set_b(_color[2]);
  markerMsg.mutable_material()->mutable_diffuse()->set_a(_color[3]);
  ignition::msgs::Set(markerMsg.add_point(), _startPoint);
  ignition::msgs::Set(markerMsg.add_point(), _endPoint);

  this->dataPtr->node.Request("/marker", markerMsg);
}

bool TapeMeasure::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::HoverToScene::kType)
  {
    auto hoverToSceneEvent =
        reinterpret_cast<gui::events::HoverToScene *>(_event);

    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->measure && hoverToSceneEvent)
    {
      math::Vector3d point = hoverToSceneEvent->Point();
      this->DrawPoint(this->dataPtr->currentId, point, this->dataPtr->hoverColor);
      if (this->dataPtr->currentId == this->dataPtr->endPointId)
      {
        this->DrawLine(this->dataPtr->lineId, this->dataPtr->startPoint, point, this->dataPtr->hoverColor);
        this->dataPtr->distance = this->dataPtr->startPoint.Distance(point);
        this->newDistance();
      }
    }
    this->dataPtr->placed = true;
  }
  // Note: the following isn't an else if statement due to the hover scene
  // event sometimes smothering this click event if the logic uses an else if
  // statement
  if (_event->type() == ignition::gazebo::gui::events::LeftClickToScene::kType)
  {
    auto leftClickToSceneEvent =
        reinterpret_cast<gui::events::LeftClickToScene *>(_event);

    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->measure && leftClickToSceneEvent)
    {
      math::Vector3d point = leftClickToSceneEvent->Point();
      this->DrawPoint(this->dataPtr->currentId, point, this->dataPtr->drawColor);
      // If we just placed the end point, end execution
      if (this->dataPtr->currentId == this->dataPtr->startPointId)
      {
        this->dataPtr->startPoint = point;
      }
      else
      {
        this->dataPtr->endPoint = point;
        this->dataPtr->measure = false;
        this->DrawLine(this->dataPtr->lineId, this->dataPtr->startPoint, this->dataPtr->endPoint, this->dataPtr->drawColor);
        ignwarn << "Distance is " << this->dataPtr->startPoint.Distance(this->dataPtr->endPoint) << "\n";
        this->dataPtr->distance = this->dataPtr->startPoint.Distance(this->dataPtr->endPoint);
        this->newDistance();
      }
      this->dataPtr->currentId = this->dataPtr->endPointId;
    }
    this->dataPtr->placed = true;
  }

  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::TapeMeasure,
                    ignition::gui::Plugin)
