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

#include "TransformControl.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Grid.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sim/gui/GuiEvents.hh"

namespace gz::sim
{
  class TransformControlPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;

    /// \brief Flag for if the snapping values should be set to the grid.
    public: bool snapToGrid{false};

    /// \brief Pointer to grid for snap to grid, assumes only one grid
    public: rendering::GridPtr grid;

    /// \brief The xyz snap values held for snap to grid.
    public: math::Vector3d xyzSnapVals{1.0, 1.0, 1.0};

    /// \brief The rpy snap values held for snap to grid.
    public: math::Vector3d rpySnapVals{45.0, 45.0, 45.0};

    /// \brief The scale snap values held for snap to grid.
    public: math::Vector3d scaleSnapVals{1.0, 1.0, 1.0};
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
TransformControl::TransformControl()
  : gz::gui::Plugin(),
  dataPtr(std::make_unique<TransformControlPrivate>())
{
}

/////////////////////////////////////////////////
TransformControl::~TransformControl() = default;

/////////////////////////////////////////////////
void TransformControl::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Transform control";

  // For transform requests
  this->dataPtr->service = "/gui/transform_mode";

  gz::gui::App()->findChild<gz::gui::MainWindow *>
      ()->installEventFilter(this);
  gz::gui::App()->findChild<gz::gui::MainWindow *>
      ()->QuickWindow()->installEventFilter(this);
}

/////////////////////////////////////////////////
void TransformControl::OnSnapUpdate(
    double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw,
    double _scaleX, double _scaleY, double _scaleZ)
{
  this->dataPtr->xyzSnapVals = math::Vector3d(_x, _y, _z);
  this->dataPtr->rpySnapVals = math::Vector3d(_roll, _pitch, _yaw);
  this->dataPtr->scaleSnapVals = math::Vector3d(_scaleX, _scaleY, _scaleZ);

  gui::events::SnapIntervals event(
      this->dataPtr->xyzSnapVals,
      this->dataPtr->rpySnapVals,
      this->dataPtr->scaleSnapVals);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(), &event);

  this->newSnapValues();
}

/////////////////////////////////////////////////
void TransformControl::OnMode(const QString &_mode)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting transform mode" << std::endl;
  };

  msgs::StringMsg req;
  req.set_data(_mode.toStdString());
  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

/////////////////////////////////////////////////
void TransformControl::OnSnapToGrid()
{
  this->dataPtr->snapToGrid = true;
}

/////////////////////////////////////////////////
void TransformControl::SnapToGrid()
{
  if (!this->dataPtr->grid)
    this->LoadGrid();

  // No grid was found, take no action
  if (!this->dataPtr->grid)
    return;

  double cellLength = this->dataPtr->grid->CellLength();
  this->OnSnapUpdate(cellLength, cellLength, cellLength,
      this->dataPtr->rpySnapVals.X(),
      this->dataPtr->rpySnapVals.Y(),
      this->dataPtr->rpySnapVals.Z(),
      this->dataPtr->scaleSnapVals.X(),
      this->dataPtr->scaleSnapVals.Y(),
      this->dataPtr->scaleSnapVals.Z());
}

/////////////////////////////////////////////////
void TransformControl::LoadGrid()
{
  auto scene = rendering::sceneFromFirstRenderEngine();

  // load grid
  // if gridPtr found, load the existing gridPtr to class
  for (unsigned int i = 0; i < scene->VisualCount(); ++i)
  {
    auto vis = scene->VisualByIndex(i);
    if (!vis || vis->GeometryCount() == 0)
      continue;
    for (unsigned int j = 0; j < vis->GeometryCount(); ++j)
    {
      auto grid = std::dynamic_pointer_cast<rendering::Grid>(
            vis->GeometryByIndex(j));
      if (grid)
      {
        this->dataPtr->grid = grid;
        return;
      }
    }
  }
}

/////////////////////////////////////////////////
bool TransformControl::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->snapToGrid)
    {
      this->SnapToGrid();
      this->dataPtr->snapToGrid = false;
    }
  }
  else if (_event->type() == QEvent::KeyPress)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent*>(_event);
    if (keyEvent->key() == Qt::Key_T)
    {
      this->activateTranslate();
    }
    else if (keyEvent->key() == Qt::Key_R)
    {
      this->activateRotate();
    }
  }
  else if (_event->type() == QEvent::KeyRelease)
  {
    QKeyEvent *keyEvent = static_cast<QKeyEvent*>(_event);
    if (keyEvent->key() == Qt::Key_Escape)
    {
      this->activateSelect();
    }
  }

  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
double TransformControl::xSnap()
{
  return this->dataPtr->xyzSnapVals[0];
}

/////////////////////////////////////////////////
double TransformControl::ySnap()
{
  return this->dataPtr->xyzSnapVals[1];
}

/////////////////////////////////////////////////
double TransformControl::zSnap()
{
  return this->dataPtr->xyzSnapVals[2];
}

/////////////////////////////////////////////////
double TransformControl::rollSnap()
{
  return this->dataPtr->rpySnapVals[0];
}

/////////////////////////////////////////////////
double TransformControl::pitchSnap()
{
  return this->dataPtr->rpySnapVals[1];
}

/////////////////////////////////////////////////
double TransformControl::yawSnap()
{
  return this->dataPtr->rpySnapVals[2];
}

/////////////////////////////////////////////////
double TransformControl::scaleXSnap()
{
  return this->dataPtr->scaleSnapVals[0];
}

/////////////////////////////////////////////////
double TransformControl::scaleYSnap()
{
  return this->dataPtr->scaleSnapVals[1];
}

/////////////////////////////////////////////////
double TransformControl::scaleZSnap()
{
  return this->dataPtr->scaleSnapVals[2];
}

// Register this plugin
IGNITION_ADD_PLUGIN(TransformControl,
                    gz::gui::Plugin)
