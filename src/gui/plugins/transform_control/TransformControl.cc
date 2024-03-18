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
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/KeyEvent.hh>
#include <gz/common/MouseEvent.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Grid.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/TransformController.hh>
#include <gz/rendering/Visual.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sim/gui/GuiEvents.hh"

namespace gz::sim
{
  class TransformControlPrivate
  {
    /// \brief Perform transformations in the render thread.
    public: void HandleTransform();

    /// \brief Handle mouse events
    public: void HandleMouseEvents();

    /// \brief Snaps a point at intervals of a fixed distance. Currently used
    /// to give a snapping behavior when moving models with a mouse.
    /// \param[in] _point Input point to snap.
    /// \param[in] _snapVals The snapping values to use for each corresponding
    /// coordinate in _point
    /// \param[in] _sensitivity Sensitivity of a point snapping, in terms of a
    /// percentage of the interval.
    public: void SnapPoint(
        gz::math::Vector3d &_point, math::Vector3d &_snapVals,
        double _sensitivity = 0.4) const;

    /// \brief Constraints the passed in axis to the currently selected axes.
    /// \param[in, out] _axis The axis to constrain.
    public: void XYZConstraint(math::Vector3d &_axis);

    /// \brief Snaps a value at intervals of a fixed distance. Currently used
    /// to give a snapping behavior when moving models with a mouse.
    /// \param[in] _coord Input coordinate point.
    /// \param[in] _interval Fixed distance interval at which the point is
    /// snapped.
    /// \param[in] _sensitivity Sensitivity of a point snapping, in terms of a
    /// percentage of the interval.
    /// \return Snapped coordinate point.
    public: double SnapValue(
        double _coord, double _interval, double _sensitivity) const;

    /// \brief Get the top level node for the given node, which
    /// is the ancestor which is a direct child to the root visual.
    /// Usually, this will be a model or a light.
    /// \param[in] _node Child node
    /// \return Top level node containining this node
    rendering::NodePtr TopLevelNode(
        const rendering::NodePtr &_node) const;

    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    // TODO(anyone): check on mutex usage
    public: std::mutex mutex;

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

    /// \brief Holds the latest mouse event
    public: gz::common::MouseEvent mouseEvent;

    /// \brief Holds the latest key event
    public: gz::common::KeyEvent keyEvent;

    /// \brief Flag to indicate whether the x key is currently being pressed
    public: bool xPressed = false;

    /// \brief Flag to indicate whether the y key is currently being pressed
    public: bool yPressed = false;

    /// \brief Flag to indicate whether the z key is currently being pressed
    public: bool zPressed = false;

    /// \brief Where the mouse left off - used to continue translating
    /// smoothly when switching axes through keybinding and clicking
    /// Updated on an x, y, or z, press or release and a mouse press
    public: math::Vector2i mousePressPos = math::Vector2i::Zero;

    /// \brief Flag to keep track of world pose setting used
    /// for button translating.
    public: bool isStartWorldPosSet = false;

    /// \brief The starting world pose of a clicked visual.
    public: gz::math::Vector3d startWorldPos = math::Vector3d::Zero;

    /// \brief Block orbit
    public: bool blockOrbit = false;

    /// \brief True if BlockOrbit events should be sent
    public: bool sendBlockOrbit = false;
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

  this->newSnapValues();
}

/////////////////////////////////////////////////
void TransformControl::OnMode(const QString &_mode)
{
  auto modeStr = _mode.toStdString();

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
    gzerr << "Unknown transform mode: [" << modeStr << "]" << std::endl;

  gz::sim::gui::events::TransformControlModeActive
    transformControlModeActive(this->dataPtr->transformMode);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &transformControlModeActive);
  this->dataPtr->mouseDirty = true;
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
  if (_event->type() == gz::gui::events::Render::kType)
  {
    // This event is called in the RenderThread, so it's safe to make
    // rendering calls here
    if (this->dataPtr->snapToGrid)
    {
      this->SnapToGrid();
      this->dataPtr->snapToGrid = false;
    }
    if (this->dataPtr->transformControl.Active())
      this->dataPtr->mouseDirty = true;
    this->dataPtr->HandleTransform();
  }
  else if (_event->type() ==
    gz::sim::gui::events::EntitiesSelected::kType)
  {
    if (!this->dataPtr->blockOrbit)
    {
      gz::sim::gui::events::EntitiesSelected *_e =
        static_cast<gz::sim::gui::events::EntitiesSelected*>(_event);
      this->dataPtr->selectedEntities = _e->Data();
    }
  }
  else if (_event->type() ==
    gz::sim::gui::events::DeselectAllEntities::kType)
  {
    if (!this->dataPtr->blockOrbit)
    {
      this->dataPtr->selectedEntities.clear();
    }
  }
  else if (_event->type() == gz::gui::events::LeftClickOnScene::kType)
  {
    gz::gui::events::LeftClickOnScene *_e =
      static_cast<gz::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == gz::gui::events::MousePressOnScene::kType)
  {
    auto event =
        static_cast<gz::gui::events::MousePressOnScene *>(_event);
    this->dataPtr->mouseEvent = event->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == gz::gui::events::DragOnScene::kType)
  {
    auto event =
        static_cast<gz::gui::events::DragOnScene *>(_event);
    this->dataPtr->mouseEvent = event->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == gz::gui::events::KeyPressOnScene::kType)
  {
    gz::gui::events::KeyPressOnScene *_e =
      static_cast<gz::gui::events::KeyPressOnScene*>(_event);
    this->dataPtr->keyEvent = _e->Key();

    if (this->dataPtr->keyEvent.Key() == Qt::Key_T)
    {
      this->activateTranslate();
    }
    else if (this->dataPtr->keyEvent.Key() == Qt::Key_R)
    {
      this->activateRotate();
    }
  }
  else if (_event->type() == gz::gui::events::KeyReleaseOnScene::kType)
  {
    gz::gui::events::KeyReleaseOnScene *_e =
      static_cast<gz::gui::events::KeyReleaseOnScene*>(_event);
    this->dataPtr->keyEvent = _e->Key();
    if (this->dataPtr->keyEvent.Key() == Qt::Key_Escape)
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

/////////////////////////////////////////////////
void TransformControlPrivate::HandleTransform()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam && cam->HasUserData("user-camera") &&
          std::get<bool>(cam->UserData("user-camera")))
      {
        this->camera = cam;
        gzdbg << "TransformControl plugin is using camera ["
               << this->camera->Name() << "]" << std::endl;
        break;
      }
    }

    if (!this->transformControl.Camera())
      this->transformControl.SetCamera(this->camera);
  }

  // set transform configuration
  this->transformControl.SetTransformMode(this->transformMode);

  // stop and detach transform controller if mode is none or no entity is
  // selected
  if (this->transformMode == rendering::TransformMode::TM_NONE ||
      (this->transformControl.Node() &&
      this->selectedEntities.empty()))
  {
    if (this->transformControl.Node())
    {
      try
      {
        this->transformControl.Node()->SetUserData(
          "pause-update", static_cast<int>(0));
      }
      catch (std::bad_variant_access &)
      {
        // It's ok to get here
      }
    }

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

  this->HandleMouseEvents();

  if (this->sendBlockOrbit)
  {
    // Events with false should only be sent once
    if (!this->blockOrbit)
      this->sendBlockOrbit = false;

    gz::gui::events::BlockOrbit blockOrbitEvent(this->blockOrbit);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
        &blockOrbitEvent);
  }
}

/////////////////////////////////////////////////
void TransformControlPrivate::HandleMouseEvents()
{
  // check for mouse events
  if (!this->mouseDirty)
    return;
  this->mouseDirty = false;

  // handle mouse movements
  if (this->mouseEvent.Button() == gz::common::MouseEvent::LEFT)
  {
    if (this->mouseEvent.Type() == gz::common::MouseEvent::PRESS
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
        if (axis != gz::math::Vector3d::Zero)
        {
          this->blockOrbit = true;
          this->sendBlockOrbit = true;
          // start the transform process
          this->transformControl.SetActiveAxis(axis);
          this->transformControl.Start();
          if (this->transformControl.Node())
          {
            try
            {
              this->transformControl.Node()->SetUserData(
                "pause-update", static_cast<int>(1));
            }
            catch (std::bad_variant_access &)
            {
              // It's ok to get here
            }
          }
        }
        else
        {
          this->blockOrbit = false;
          this->sendBlockOrbit = true;
          return;
        }
      }
    }
    else if (this->mouseEvent.Type() == gz::common::MouseEvent::RELEASE)
    {
      this->blockOrbit = false;
      this->sendBlockOrbit = true;

      this->isStartWorldPosSet = false;
      if (this->transformControl.Active())
      {
        if (this->transformControl.Node())
        {
          std::function<void(const gz::msgs::Boolean &, const bool)> cb =
            [this](const gz::msgs::Boolean &/*_rep*/, const bool _result)
          {
            if (this->transformControl.Node())
            {
              try
              {
                this->transformControl.Node()->SetUserData(
                  "pause-update", static_cast<int>(0));
              }
              catch (std::bad_variant_access &)
              {
                // It's ok to get here
              }
            }
            if (!_result)
              gzerr << "Error setting pose" << std::endl;
          };
          rendering::NodePtr nodeTmp = this->transformControl.Node();
          auto topVisual = std::dynamic_pointer_cast<rendering::Visual>(
            nodeTmp);
          gz::msgs::Pose req;
          req.set_name(topVisual->Name());
          msgs::Set(req.mutable_position(), nodeTmp->WorldPosition());
          msgs::Set(req.mutable_orientation(), nodeTmp->WorldRotation());

          // First time, create the service
          if (this->poseCmdService.empty())
          {
            std::string worldName;
            auto worldNames = gz::gui::worldNames();
            if (!worldNames.empty())
              worldName = worldNames[0].toStdString();

            this->poseCmdService = "/world/" + worldName + "/set_pose";

            this->poseCmdService = transport::TopicUtils::AsValidTopic(
                this->poseCmdService);
            if (this->poseCmdService.empty())
            {
              gzerr << "Failed to create valid pose command service "
                     << "for world [" << worldName << "]" << std::endl;
              return;
            }
          }
          this->node.Request(this->poseCmdService, req, cb);
        }

        this->transformControl.Stop();
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
        if (axis == gz::math::Vector3d::Zero)
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
            // Highlight entity and notify other widgets

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
                try
                {
                  topClickedVisual->SetUserData(
                    "pause-update", static_cast<int>(1));
                }
                catch (std::bad_variant_access &)
                {
                  // It's ok to get here
                }
              }
              else
              {
                this->transformControl.Detach();
                try
                {
                  topClickedVisual->SetUserData(
                    "pause-update", static_cast<int>(0));
                }
                catch (std::bad_variant_access &)
                {
                  // It's ok to get here
                }
              }
            }

            return;
          }
        }
      }
    }
  }
  if (this->mouseEvent.Type() == common::MouseEvent::MOVE
      && this->transformControl.Active())
  {
    if (this->transformControl.Node()){
      try
      {
        this->transformControl.Node()->SetUserData(
          "pause-update", static_cast<int>(1));
      } catch (std::bad_variant_access &)
      {
        // It's ok to get here
      }
    }

    this->blockOrbit = true;
    this->sendBlockOrbit = true;
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
        auto entityId = kNullEntity;
        try
        {
          entityId = std::get<uint64_t>(visual->UserData("gazebo-entity"));
        }
        catch (std::bad_variant_access &)
        {
          // It's ok to get here
        }
        if (entityId == nodeId)
        {
          target = std::dynamic_pointer_cast<rendering::Node>(
            this->scene->VisualById(visual->Id()));
          break;
        }
      }
      if (!target)
      {
        gzwarn << "Failed to find node with ID [" << nodeId << "]"
                << std::endl;
        return;
      }
      this->XYZConstraint(axis);
      if (!this->isStartWorldPosSet)
      {
        this->isStartWorldPosSet = true;
        this->startWorldPos = target->WorldPosition();
      }
      math::Vector3d distance =
        this->transformControl.TranslationFrom2d(axis, start, end);
      if (this->keyEvent.Control())
      {
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
          snapVals.X() = GZ_PI/4;
        }
        else
        {
          snapVals.X() = GZ_DTOR(snapVals.X());
        }
        if (snapVals.Y() <= 1e-4)
        {
          snapVals.Y() = GZ_PI/4;
        }
        else
        {
          snapVals.Y() = GZ_DTOR(snapVals.Y());
        }
        if (snapVals.Z() <= 1e-4)
        {
          snapVals.Z() = GZ_PI/4;
        }
        else
        {
          snapVals.Z() = GZ_DTOR(snapVals.Z());
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
  }
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
    gz::math::Vector3d &_point, math::Vector3d &_snapVals,
    double _sensitivity) const
{
  if (_snapVals.X() <= 0 || _snapVals.Y() <= 0 || _snapVals.Z() <= 0)
  {
    gzerr << "Interval distance must be greater than 0"
        << std::endl;
    return;
  }

  if (_sensitivity < 0 || _sensitivity > 1.0)
  {
    gzerr << "Sensitivity must be between 0 and 1" << std::endl;
    return;
  }

  _point.X() = this->SnapValue(_point.X(), _snapVals.X(), _sensitivity);
  _point.Y() = this->SnapValue(_point.Y(), _snapVals.Y(), _sensitivity);
  _point.Z() = this->SnapValue(_point.Z(), _snapVals.Z(), _sensitivity);
}

// Register this plugin
GZ_ADD_PLUGIN(TransformControl,
              gz::gui::Plugin)
