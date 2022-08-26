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

#include <ignition/msgs/marker.pb.h>

#include <iostream>
#include <unordered_set>
#include <string>
#include <memory>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <ignition/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sim/gui/GuiEvents.hh"
#include "TapeMeasure.hh"

namespace ignition::gazebo
{
  class TapeMeasurePrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief True if currently measuring, else false.
    public: bool measure = false;

    /// \brief The id of the start point marker.
    public: const int kStartPointId = 1;

    /// \brief The id of the end point marker.
    public: const int kEndPointId = 2;

    /// \brief The id of the line marker.
    public: const int kLineId = 3;

    /// \brief The id of the start or end point marker that is currently
    /// being placed. This is primarily used to track the state machine of
    /// the plugin.
    public: int currentId = kStartPointId;

    /// \brief The location of the placed starting point of the tape measure
    /// tool, only set when the user clicks to set the point.
    public: gz::math::Vector3d startPoint =
            gz::math::Vector3d::Zero;

    /// \brief The location of the placed ending point of the tape measure
    /// tool, only set when the user clicks to set the point.
    public: gz::math::Vector3d endPoint = gz::math::Vector3d::Zero;

    /// \brief The color to set the marker when hovering the mouse over the
    /// scene.
    public: gz::math::Color
            hoverColor{gz::math::Color(0.2, 0.2, 0.2, 0.5)};

    /// \brief The color to draw the marker when the user clicks to confirm
    /// its location.
    public: gz::math::Color
            drawColor{gz::math::Color(0.2, 0.2, 0.2, 1.0)};

    /// \brief A set of the currently placed markers.  Used to make sure a
    /// non-existent marker is not deleted.
    public: std::unordered_set<int> placedMarkers;

    /// \brief The current distance between the two points.  This distance
    /// is updated as the user hovers the end point as well.
    public: double distance = 0.0;

    /// \brief The namespace that the markers for this plugin are placed in.
    public: std::string ns = "tape_measure";
  };
}

using namespace ignition;
using namespace ignition::gazebo;

/////////////////////////////////////////////////
TapeMeasure::TapeMeasure()
  : gz::gui::Plugin(),
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

  gz::gui::App()->findChild<gz::gui::MainWindow *>
      ()->installEventFilter(this);
  gz::gui::App()->findChild<gz::gui::MainWindow *>
      ()->QuickWindow()->installEventFilter(this);
}

/////////////////////////////////////////////////
void TapeMeasure::OnMeasure()
{
  this->Measure();
}

/////////////////////////////////////////////////
void TapeMeasure::Measure()
{
  this->Reset();
  this->dataPtr->measure = true;
  QGuiApplication::setOverrideCursor(Qt::CrossCursor);

  // Notify Scene3D to disable the right click menu while we use it to
  // cancel our current measuring action
  gz::gui::events::DropdownMenuEnabled dropdownMenuEnabledEvent(false);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &dropdownMenuEnabledEvent);
}

/////////////////////////////////////////////////
void TapeMeasure::OnReset()
{
  this->Reset();
}

/////////////////////////////////////////////////
void TapeMeasure::Reset()
{
  this->DeleteMarker(this->dataPtr->kStartPointId);
  this->DeleteMarker(this->dataPtr->kEndPointId);
  this->DeleteMarker(this->dataPtr->kLineId);

  this->dataPtr->currentId = this->dataPtr->kStartPointId;
  this->dataPtr->startPoint = math::Vector3d::Zero;
  this->dataPtr->endPoint = math::Vector3d::Zero;
  this->dataPtr->distance = 0.0;
  this->dataPtr->measure = false;
  this->newDistance();
  QGuiApplication::restoreOverrideCursor();

  // Notify Scene3D that we are done using the right click, so it can
  // re-enable the settings menu
  gz::gui::events::DropdownMenuEnabled dropdownMenuEnabledEvent(true);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &dropdownMenuEnabledEvent);
}

/////////////////////////////////////////////////
double TapeMeasure::Distance()
{
  return this->dataPtr->distance;
}

/////////////////////////////////////////////////
void TapeMeasure::DeleteMarker(int _id)
{
  if (this->dataPtr->placedMarkers.find(_id) ==
      this->dataPtr->placedMarkers.end())
    return;

  // Delete the previously created marker
  msgs::Marker markerMsg;
  markerMsg.set_ns(this->dataPtr->ns);
  markerMsg.set_id(_id);
  markerMsg.set_action(msgs::Marker::DELETE_MARKER);
  this->dataPtr->node.Request("/marker", markerMsg);
  this->dataPtr->placedMarkers.erase(_id);
}

/////////////////////////////////////////////////
void TapeMeasure::DrawPoint(int _id,
    math::Vector3d &_point, math::Color &_color)
{
  this->DeleteMarker(_id);

  msgs::Marker markerMsg;
  markerMsg.set_ns(this->dataPtr->ns);
  markerMsg.set_id(_id);
  markerMsg.set_action(msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(msgs::Marker::SPHERE);
  msgs::Set(markerMsg.mutable_material()->mutable_ambient(), _color);
  msgs::Set(markerMsg.mutable_material()->mutable_diffuse(), _color);
  msgs::Set(markerMsg.mutable_scale(),
    math::Vector3d(0.1, 0.1, 0.1));
  msgs::Set(markerMsg.mutable_pose(),
    math::Pose3d(_point.X(), _point.Y(), _point.Z(), 0, 0, 0));

  this->dataPtr->node.Request("/marker", markerMsg);
  this->dataPtr->placedMarkers.insert(_id);
}

/////////////////////////////////////////////////
void TapeMeasure::DrawLine(int _id, math::Vector3d &_startPoint,
    math::Vector3d &_endPoint, math::Color &_color)
{
  this->DeleteMarker(_id);

  msgs::Marker markerMsg;
  markerMsg.set_ns(this->dataPtr->ns);
  markerMsg.set_id(_id);
  markerMsg.set_action(msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(msgs::Marker::LINE_LIST);
  msgs::Set(markerMsg.mutable_material()->mutable_ambient(), _color);
  msgs::Set(markerMsg.mutable_material()->mutable_diffuse(), _color);
  msgs::Set(markerMsg.add_point(), _startPoint);
  msgs::Set(markerMsg.add_point(), _endPoint);

  this->dataPtr->node.Request("/marker", markerMsg);
  this->dataPtr->placedMarkers.insert(_id);
}

/////////////////////////////////////////////////
bool TapeMeasure::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::HoverToScene::kType)
  {
    auto hoverToSceneEvent =
        reinterpret_cast<gz::gui::events::HoverToScene *>(_event);

    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->measure && hoverToSceneEvent)
    {
      math::Vector3d point = hoverToSceneEvent->Point();
      this->DrawPoint(this->dataPtr->currentId, point,
        this->dataPtr->hoverColor);

      // If the user is currently choosing the end point, draw the connecting
      // line and update the new distance.
      if (this->dataPtr->currentId == this->dataPtr->kEndPointId)
      {
        this->DrawLine(this->dataPtr->kLineId, this->dataPtr->startPoint,
          point, this->dataPtr->hoverColor);
        this->dataPtr->distance = this->dataPtr->startPoint.Distance(point);
        this->newDistance();
      }
    }
  }
  else if (_event->type() == gz::gui::events::LeftClickToScene::kType)
  {
    auto leftClickToSceneEvent =
        reinterpret_cast<gz::gui::events::LeftClickToScene *>(_event);

    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->measure && leftClickToSceneEvent)
    {
      math::Vector3d point = leftClickToSceneEvent->Point();
      this->DrawPoint(this->dataPtr->currentId, point,
        this->dataPtr->drawColor);
      // If the user is placing the start point, update its position
      if (this->dataPtr->currentId == this->dataPtr->kStartPointId)
      {
        this->dataPtr->startPoint = point;
      }
      // If the user is placing the end point, update the end position,
      // end the measurement state, and update the draw line and distance
      else
      {
        this->dataPtr->endPoint = point;
        this->dataPtr->measure = false;
        this->DrawLine(this->dataPtr->kLineId, this->dataPtr->startPoint,
          this->dataPtr->endPoint, this->dataPtr->drawColor);
        this->dataPtr->distance =
          this->dataPtr->startPoint.Distance(this->dataPtr->endPoint);
        this->newDistance();
        QGuiApplication::restoreOverrideCursor();

        // Notify Scene3D that we are done using the right click, so it can
        // re-enable the settings menu
        gz::gui::events::DropdownMenuEnabled
          dropdownMenuEnabledEvent(true);

        gz::gui::App()->sendEvent(
            gz::gui::App()->findChild<gz::gui::MainWindow *>(),
            &dropdownMenuEnabledEvent);
      }
      this->dataPtr->currentId = this->dataPtr->kEndPointId;
    }
  }
  else if (_event->type() == QEvent::KeyPress)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent*>(_event);
    if (keyEvent && keyEvent->key() == Qt::Key_M)
    {
      this->Reset();
      this->Measure();
    }
  }
  else if (_event->type() == QEvent::KeyRelease)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent*>(_event);
    if (keyEvent && keyEvent->key() == Qt::Key_Escape &&
        this->dataPtr->measure)
    {
      this->Reset();
    }
  }
  // Cancel the current action if a right click is detected
  else if (_event->type() == gz::gui::events::RightClickToScene::kType)
  {
    if (this->dataPtr->measure)
    {
      this->Reset();
    }
  }

  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(TapeMeasure,
                    gz::gui::Plugin)
