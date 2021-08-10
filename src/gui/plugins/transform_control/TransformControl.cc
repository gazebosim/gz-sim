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

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/KeyEvent.hh>
#include <ignition/common/MouseEvent.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/Helpers.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/Geometry.hh>
#include <ignition/rendering/Grid.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering/TransformController.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/gui/GuiEvents.hh"

// TODO: snap not working
// TODO: select -> press T: no transform

namespace ignition::gazebo
{
  class TransformControlPrivate
  {
    /// \brief Perform transformations in the render thread.
    public: void HandleTransform();

    /// \brief Snaps a point at intervals of a fixed distance. Currently used
    /// to give a snapping behavior when moving models with a mouse.
    /// \param[in] _point Input point to snap.
    /// \param[in] _snapVals The snapping values to use for each corresponding
    /// coordinate in _point
    /// \param[in] _sensitivity Sensitivity of a point snapping, in terms of a
    /// percentage of the interval.
    public: void SnapPoint(
        ignition::math::Vector3d &_point, math::Vector3d &_snapVals,
        double _sensitivity = 0.4) const;

    /// \brief Constraints the passed in axis to the currently selected axes.
    /// \param[in] _axis The axis to constrain.
    public: void XYZConstraint(math::Vector3d &_axis);

    public: double SnapValue(
        double _coord, double _interval, double _sensitivity) const;

    /// \brief Get the top level node for the given node, which
    /// is the ancestor which is a direct child to the root visual.
    /// Usually, this will be a model or a light.
    /// \param[in] _node Child node
    /// \return Top level node containining this node
    rendering::NodePtr TopLevelNode(
        const rendering::NodePtr &_node) const;

    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
// TODO: check on mutex usage
    public: std::mutex mutex;

    /// \brief Transform control service name
    /// \brief Only used when in legacy mode, where this plugin requested a
    /// transport service provided by `GzScene3D`.
    /// The new behaviour is that this plugin performs the entire transform
    /// operation.
    public: std::string service{"/gui/transform_mode"};

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

    /// \brief Transform mode: none, translation, rotation, or scale
    public: rendering::TransformMode transformMode =
        rendering::TransformMode::TM_NONE;

    /// \brief Transform space: local or world
    public: rendering::TransformSpace transformSpace =
        rendering::TransformSpace::TS_LOCAL;

    /// \brief Transform controller for models
    public: rendering::TransformController transformControl;

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief True if there are new mouse events to process.
    public: bool mouseDirty{false};

    /// \brief Whether the transform gizmo is being dragged.
    public: bool transformActive{false};

    /// \brief Name of service for setting entity pose
    public: std::string poseCmdService;

    /// \brief Currently selected entities, organized by order of selection.
    public: std::vector<Entity> selectedEntities;

    public: ignition::common::MouseEvent mouseEvent;

    /// \brief Flag to indicate whether the x key is currently being pressed
    public: bool xPressed = false;

    /// \brief Flag to indicate whether the y key is currently being pressed
    public: bool yPressed = false;

    /// \brief Flag to indicate whether the z key is currently being pressed
    public: bool zPressed = false;

    /// \brief Flag to indicate whether the escape key has been released.
    public: bool escapeReleased = false;

    /// \brief Where the mouse left off - used to continue translating
    /// smoothly when switching axes through keybinding and clicking
    /// Updated on an x, y, or z, press or release and a mouse press
    public: math::Vector2i mousePressPos = math::Vector2i::Zero;

    /// \brief Flag to keep track of world pose setting used
    /// for button translating.
    public: bool isStartWorldPosSet = false;

    /// \brief The starting world pose of a clicked visual.
    public: ignition::math::Vector3d startWorldPos = math::Vector3d::Zero;

    public: ignition::common::KeyEvent keyEvent;

    public: bool blockOrbit = false;

    /// \brief Enable legacy features for plugin to work with GzScene3D.
    /// Disable them to work with the new MinimalScene plugin.
    public: bool legacy{true};
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TransformControl::TransformControl()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<TransformControlPrivate>())
{
}

/////////////////////////////////////////////////
TransformControl::~TransformControl() = default;

/////////////////////////////////////////////////
void TransformControl::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Transform control";

  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("legacy"))
    {
      elem->QueryBoolText(&this->dataPtr->legacy);
    }
  }

  if (this->dataPtr->legacy)
  {
    igndbg << "Legacy mode is enabled; this plugin must be used with "
           << "GzScene3D." << std::endl;
  }
  else
  {
    igndbg << "Legacy mode is disabled; this plugin must be used with "
           << "MinimalScene." << std::endl;
  }

  ignition::gui::App()->findChild<ignition::gui::MainWindow *>
      ()->installEventFilter(this);
  ignition::gui::App()->findChild<ignition::gui::MainWindow *>
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

  // Emit event to GzScene3D in legacy mode
  if (this->dataPtr->legacy)
  {
    ignition::gui::events::SnapIntervals event(
        this->dataPtr->xyzSnapVals,
        this->dataPtr->rpySnapVals,
        this->dataPtr->scaleSnapVals);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(), &event);
  }

  this->newSnapValues();
}

/////////////////////////////////////////////////
void TransformControl::OnMode(const QString &_mode)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting transform mode" << std::endl;
  };

  auto modeStr = _mode.toStdString();

  // Legacy behaviour: send request to GzScene3D
  if (this->dataPtr->legacy)
  {
    ignition::msgs::StringMsg req;
    req.set_data(modeStr);
    this->dataPtr->node.Request(this->dataPtr->service, req, cb);
  }
  // New behaviour: handle the transform control locally
  else
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    if (modeStr == "select")
      this->dataPtr->transformMode = rendering::TransformMode::TM_NONE;
    else if (modeStr == "translate")
      this->dataPtr->transformMode = rendering::TransformMode::TM_TRANSLATION;
    else if (modeStr == "rotate")
      this->dataPtr->transformMode = rendering::TransformMode::TM_ROTATION;
    else if (modeStr == "scale")
      this->dataPtr->transformMode = rendering::TransformMode::TM_SCALE;
    else
      ignerr << "Unknown transform mode: [" << modeStr << "]" << std::endl;

    ignition::gazebo::gui::events::TransformControlMode transformControlMode(
      this->dataPtr->transformMode);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &transformControlMode);
  }
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
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    // This event is called in Scene3d's RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->snapToGrid)
    {
      this->SnapToGrid();
      this->dataPtr->snapToGrid = false;
    }
    this->dataPtr->HandleTransform();
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
  else if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    ignition::gazebo::gui::events::EntitiesSelected *_e =
      static_cast<ignition::gazebo::gui::events::EntitiesSelected*>(_event);
    this->dataPtr->selectedEntities = _e->Data();
  }
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    if (this->dataPtr->transformControl.Active())
      this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() ==
    ignition::gazebo::gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntities.clear();
  }
  else if (_event->type() == ignition::gui::events::LeftClickOnScene::kType)
  {
    ignition::gui::events::LeftClickOnScene *_e =
      static_cast<ignition::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == ignition::gui::events::KeyPressOnScene::kType)
  {
    ignition::gui::events::KeyPressOnScene *_e =
      static_cast<ignition::gui::events::KeyPressOnScene*>(_event);
    this->dataPtr->keyEvent = _e->Key();
ignerr << "EVENT " << this->dataPtr->keyEvent.Control() << std::endl;
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

/////////////////////////////////////////////////
void TransformControlPrivate::HandleTransform()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
      return;

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam)
      {
        if (cam->Name().find("scene::Camera") != std::string::npos)
        {
          this->camera = cam;
          igndbg << "TransformControl plugin is using camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }

    if (!this->transformControl.Camera())
      this->transformControl.SetCamera(this->camera);
  }

  // Escape action, clear all selections
  if (this->escapeReleased)
  {
    this->selectedEntities.clear();
    this->escapeReleased = false;
  }

  // set transform configuration
  this->transformControl.SetTransformMode(this->transformMode);

  // stop and detach transform controller if mode is none or no entity is
  // selected
  if (this->transformMode == rendering::TransformMode::TM_NONE ||
      (this->transformControl.Node() &&
      this->selectedEntities.empty()))
  {
    if (this->transformControl.Active())
      this->transformControl.Stop();

    this->transformControl.Detach();
  }
  else
  {
    // shift indicates world space transformation
    this->transformSpace = (this->keyEvent.Shift()) ?
        rendering::TransformSpace::TS_WORLD :
        rendering::TransformSpace::TS_LOCAL;
    this->transformControl.SetTransformSpace(
        this->transformSpace);
  }

  // update gizmo visual
  this->transformControl.Update();

  // check for mouse events
  if (!this->mouseDirty)
    return;

  // handle mouse movements
  if (this->mouseEvent.Button() == ignition::common::MouseEvent::LEFT)
  {
    if (this->mouseEvent.Type() == ignition::common::MouseEvent::PRESS
        && this->transformControl.Node())
    {
      this->mousePressPos = this->mouseEvent.Pos();

      // get the visual at mouse position
      rendering::VisualPtr visual = this->scene->VisualAt(
            this->camera,
            this->mouseEvent.Pos());

      if (visual)
      {
        // check if the visual is an axis in the gizmo visual
        math::Vector3d axis =
            this->transformControl.AxisById(visual->Id());
        if (axis != ignition::math::Vector3d::Zero)
        {
          this->blockOrbit = true;
          // start the transform process
          this->transformControl.SetActiveAxis(axis);
          this->transformControl.Start();
          this->mouseDirty = false;
        }
        else
        {
          this->blockOrbit = false;
          return;
        }
      }
    }
    else if (this->mouseEvent.Type() == ignition::common::MouseEvent::RELEASE)
    {
      this->blockOrbit = false;

      this->isStartWorldPosSet = false;
      if (this->transformControl.Active())
      {
        if (this->transformControl.Node())
        {
          std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
              [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
          {
            if (!_result)
              ignerr << "Error setting pose" << std::endl;
          };
          rendering::NodePtr nodeTmp = this->transformControl.Node();
          auto topVisual = std::dynamic_pointer_cast<rendering::Visual>(
            nodeTmp);
          ignition::msgs::Pose req;
          req.set_name(topVisual->Name());
          msgs::Set(req.mutable_position(), nodeTmp->WorldPosition());
          msgs::Set(req.mutable_orientation(), nodeTmp->WorldRotation());

          // First time, create the service
          if (this->poseCmdService.empty())
          {
            std::string worldName;
            auto worldNames = ignition::gui::worldNames();
            if (!worldNames.empty())
              worldName = worldNames[0].toStdString();

            this->poseCmdService = "/world/" + worldName + "/set_pose";

            this->poseCmdService = transport::TopicUtils::AsValidTopic(
                this->poseCmdService);
            if (this->poseCmdService.empty())
            {
              ignerr << "Failed to create valid pose command service for world ["
                     << worldName <<"]" << std::endl;
              return;
            }
          }
          this->node.Request(this->poseCmdService, req, cb);
        }

        this->transformControl.Stop();
        this->mouseDirty = false;
      }
      // Select entity
      else if (!this->mouseEvent.Dragging())
      {
        rendering::VisualPtr visual = this->scene->VisualAt(
              this->camera,
              this->mouseEvent.Pos());

        if (!visual)
        {
          return;
        }

        // check if the visual is an axis in the gizmo visual
        math::Vector3d axis = this->transformControl.AxisById(visual->Id());
        if (axis == ignition::math::Vector3d::Zero)
        {
          auto topNode = this->TopLevelNode(visual);
          if (!topNode)
          {
            return;
          }

          auto topVis = std::dynamic_pointer_cast<rendering::Visual>(topNode);
          // TODO(anyone) Check plane geometry instead of hardcoded name!
          if (topVis && topVis->Name() != "ground_plane")
          {
            // // Highlight entity and notify other widgets

            // Attach control if in a transform mode - control is attached to:
            // * latest selection
            // * top-level nodes (model, light...)
            if (this->transformMode != rendering::TransformMode::TM_NONE)
            {
              rendering::VisualPtr clickedVisual = this->scene->VisualAt(
                    this->camera,
                    this->mouseEvent.Pos());

              auto topClickedNode = this->TopLevelNode(clickedVisual);
              auto topClickedVisual =
                std::dynamic_pointer_cast<rendering::Visual>(topClickedNode);

              if (topClickedNode == topClickedVisual)
              {
                this->transformControl.Attach(topClickedVisual);
              }
              else
              {
                this->transformControl.Detach();
              }
            }

            this->mouseDirty = false;
            return;
          }
        }
      }
    }
  }
  if (this->mouseEvent.Type() == common::MouseEvent::MOVE
      && this->transformControl.Active())
  {
    this->blockOrbit = true;
    // compute the the start and end mouse positions in normalized coordinates
    auto imageWidth = static_cast<double>(this->camera->ImageWidth());
    auto imageHeight = static_cast<double>(
        this->camera->ImageHeight());
    double nx = 2.0 * this->mousePressPos.X() / imageWidth - 1.0;
    double ny = 1.0 - 2.0 * this->mousePressPos.Y() / imageHeight;
    double nxEnd = 2.0 * this->mouseEvent.Pos().X() / imageWidth - 1.0;
    double nyEnd = 1.0 - 2.0 * this->mouseEvent.Pos().Y() / imageHeight;
    math::Vector2d start(nx, ny);
    math::Vector2d end(nxEnd, nyEnd);

    // get the current active axis
    math::Vector3d axis = this->transformControl.ActiveAxis();

    // compute 3d transformation from 2d mouse movement
    if (this->transformControl.Mode() ==
        rendering::TransformMode::TM_TRANSLATION)
    {
      Entity nodeId = this->selectedEntities.front();
      rendering::NodePtr target;
      for (unsigned int i = 0; i < this->scene->VisualCount(); i++)
      {
        auto visual = this->scene->VisualByIndex(i);
        auto entityId = static_cast<unsigned int>(
            std::get<int>(visual->UserData("gazebo-entity")));
        if (entityId == nodeId)
        {
          target = std::dynamic_pointer_cast<rendering::Node>(
            this->scene->VisualById(visual->Id()));
          break;
        }
      }
      if (!target)
      {
        ignwarn << "Failed to find node with ID [" << nodeId << "]"
                << std::endl;
        return;
      }
      this->XYZConstraint(axis);
      if (!this->isStartWorldPosSet)
      {
        this->isStartWorldPosSet = true;
        this->startWorldPos = target->WorldPosition();
      }
      ignition::math::Vector3d worldPos = target->WorldPosition();
      math::Vector3d distance =
        this->transformControl.TranslationFrom2d(axis, start, end);
ignerr << "CONTROL " << this->keyEvent.Control() << std::endl;
      if (this->keyEvent.Control())
      {
ignerr << "SNAP" << std::endl;
        // Translate to world frame for snapping
        distance += this->startWorldPos;
        math::Vector3d snapVals = this->xyzSnapVals;

        // Constrain snap values to a minimum of 1e-4
        snapVals.X() = std::max(1e-4, snapVals.X());
        snapVals.Y() = std::max(1e-4, snapVals.Y());
        snapVals.Z() = std::max(1e-4, snapVals.Z());

        this->SnapPoint(distance, snapVals);

        // Translate back to entity frame
        distance -= this->startWorldPos;
        distance *= axis;
      }
      this->transformControl.Translate(distance);
    }
    else if (this->transformControl.Mode() ==
        rendering::TransformMode::TM_ROTATION)
    {
      math::Quaterniond rotation =
          this->transformControl.RotationFrom2d(axis, start, end);

      if (this->keyEvent.Control())
      {
        math::Vector3d currentRot = rotation.Euler();
        math::Vector3d snapVals = this->rpySnapVals;

        if (snapVals.X() <= 1e-4)
        {
          snapVals.X() = IGN_PI/4;
        }
        else
        {
          snapVals.X() = IGN_DTOR(snapVals.X());
        }
        if (snapVals.Y() <= 1e-4)
        {
          snapVals.Y() = IGN_PI/4;
        }
        else
        {
          snapVals.Y() = IGN_DTOR(snapVals.Y());
        }
        if (snapVals.Z() <= 1e-4)
        {
          snapVals.Z() = IGN_PI/4;
        }
        else
        {
          snapVals.Z() = IGN_DTOR(snapVals.Z());
        }

        this->SnapPoint(currentRot, snapVals);
        rotation = math::Quaterniond::EulerToQuaternion(currentRot);
      }
      this->transformControl.Rotate(rotation);
    }
    else if (this->transformControl.Mode() ==
        rendering::TransformMode::TM_SCALE)
    {
      this->XYZConstraint(axis);
      // note: scaling is limited to local space
      math::Vector3d scale =
          this->transformControl.ScaleFrom2d(axis, start, end);
      if (this->keyEvent.Control())
      {
        math::Vector3d snapVals = this->scaleSnapVals;

        if (snapVals.X() <= 1e-4)
          snapVals.X() = 0.1;
        if (snapVals.Y() <= 1e-4)
          snapVals.Y() = 0.1;
        if (snapVals.Z() <= 1e-4)
          snapVals.Z() = 0.1;

        this->SnapPoint(scale, snapVals);
      }
      this->transformControl.Scale(scale);
    }
    this->mouseDirty = false;
  }

  ignition::gui::events::BlockOrbit blockOrbitEvent(this->blockOrbit);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &blockOrbitEvent);
}

/////////////////////////////////////////////////
rendering::NodePtr TransformControlPrivate::TopLevelNode(
    const rendering::NodePtr &_node) const
{
  if (!this->scene)
    return rendering::NodePtr();

  rendering::NodePtr rootNode = this->scene->RootVisual();

  rendering::NodePtr nodeTmp = _node;
  while (nodeTmp && nodeTmp->Parent() != rootNode)
  {
    nodeTmp =
      std::dynamic_pointer_cast<rendering::Node>(nodeTmp->Parent());
  }

  return nodeTmp;
}

/////////////////////////////////////////////////
double TransformControlPrivate::SnapValue(
    double _coord, double _interval, double _sensitivity) const
{
  double snap = _interval * _sensitivity;
  double rem = fmod(_coord, _interval);
  double minInterval = _coord - rem;

  if (rem < 0)
  {
    minInterval -= _interval;
  }

  double maxInterval = minInterval + _interval;

  if (_coord < (minInterval + snap))
  {
    _coord = minInterval;
  }
  else if (_coord > (maxInterval - snap))
  {
    _coord = maxInterval;
  }

  return _coord;
}

/////////////////////////////////////////////////
void TransformControlPrivate::XYZConstraint(math::Vector3d &_axis)
{
  math::Vector3d translationAxis = math::Vector3d::Zero;

  if (this->xPressed)
  {
    translationAxis += math::Vector3d::UnitX;
  }

  if (this->yPressed)
  {
    translationAxis += math::Vector3d::UnitY;
  }

  if (this->zPressed)
  {
    translationAxis += math::Vector3d::UnitZ;
  }

  if (translationAxis != math::Vector3d::Zero)
  {
    _axis = translationAxis;
  }
}

/////////////////////////////////////////////////
void TransformControlPrivate::SnapPoint(
    ignition::math::Vector3d &_point, math::Vector3d &_snapVals,
    double _sensitivity) const
{
  if (_snapVals.X() <= 0 || _snapVals.Y() <= 0 || _snapVals.Z() <= 0)
  {
    ignerr << "Interval distance must be greater than 0"
        << std::endl;
    return;
  }

  if (_sensitivity < 0 || _sensitivity > 1.0)
  {
    ignerr << "Sensitivity must be between 0 and 1" << std::endl;
    return;
  }

  _point.X() = this->SnapValue(_point.X(), _snapVals.X(), _sensitivity);
  _point.Y() = this->SnapValue(_point.Y(), _snapVals.Y(), _sensitivity);
  _point.Z() = this->SnapValue(_point.Z(), _snapVals.Z(), _sensitivity);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::TransformControl,
                    ignition::gui::Plugin)
