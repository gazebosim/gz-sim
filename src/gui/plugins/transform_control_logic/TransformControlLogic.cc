/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/KeyEvent.hh>
#include <ignition/common/MouseEvent.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>

#include <ignition/math/Quaternion.hh>

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <ignition/transport/Node.hh>
#include <ignition/rendering/TransformController.hh>

#include "ignition/gazebo/gui/GuiEvents.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"

#include "TransformControlLogic.hh"

/// \brief Private data class for TransformControlLogic
class ignition::gazebo::plugins::TransformControlLogicPrivate
{
  /// \brief Callback for a transform mode request
  /// \param[in] _msg Request message to set a new transform mode
  /// \param[in] _res Response data
  /// \return True if the request is received
  public: bool OnTransformMode(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

  /// \brief The method handle the logic of the transform control.
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

  /// \brief Set the XYZ snap values.
  /// \param[in] _xyz The XYZ snap values
  public: void SetXYZSnap(const math::Vector3d &_xyz);

  /// \brief Set the RPY snap values.
  /// \param[in] _rpy The RPY snap values
  public: void SetRPYSnap(const math::Vector3d &_rpy);

  /// \brief Set the scale snap values.
  /// \param[in] _scale The scale snap values
  public: void SetScaleSnap(const math::Vector3d &_scale);

  /// \brief Get the top level node for the given node, which
  /// is the ancestor which is a direct child to the root visual.
  /// Usually, this will be a model or a light.
  /// \param[in] _node Child node
  /// \return Top level node containining this node
  rendering::NodePtr TopLevelNode(
      const rendering::NodePtr &_node) const;

  /// \brief Whether the transform gizmo is being dragged.
  public: bool transformActive{false};

  /// \brief Transform mode service
  public: std::string transformModeService;

  /// \brief Transport node
  public: transport::Node node;

  /// \brief Name of service for setting entity pose
  public: std::string poseCmdService;

  /// \brief Name of the world
  public: std::string worldName;

  /// \brief Transform mode: none, translation, rotation, or scale
  public: rendering::TransformMode transformMode =
      rendering::TransformMode::TM_NONE;

  /// \brief Transform space: local or world
  public: rendering::TransformSpace transformSpace =
      rendering::TransformSpace::TS_LOCAL;

  /// \brief Currently selected entities, organized by order of selection.
  public: std::vector<Entity> selectedEntities;

  /// \brief Transform controller for models
  public: rendering::TransformController transformControl;

  public: ignition::common::MouseEvent mouseEvent;

  public: bool mouseDirty = false;

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
  public: ignition::math::Vector3d startWorldPos = math::Vector3d::Zero;

  public: ignition::common::KeyEvent keyEvent;

  public: std::mutex mutex;

  //// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene{nullptr};

  /// \brief User camera
  public: rendering::CameraPtr camera{nullptr};

  /// \brief The xyz values by which to snap the object.
  public: math::Vector3d xyzSnap = math::Vector3d::One;

  /// \brief The rpy values by which to snap the object.
  public: math::Vector3d rpySnap = {45, 45, 45};

  /// \brief The scale values by which to snap the object.
  public: math::Vector3d scaleSnap = math::Vector3d::One;

  public: bool blockOrbit = false;
};

using namespace ignition;
using namespace gazebo;
using namespace plugins;

void TransformControlLogicPrivate::HandleTransform()
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
          igndbg << "InteractiveViewControl plugin is moving camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
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

  // handle transform control
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
          if (this->poseCmdService.empty())
          {
            this->poseCmdService = "/world/" + this->worldName
                + "/set_pose";
          }
          this->poseCmdService = transport::TopicUtils::AsValidTopic(
              this->poseCmdService);
          if (this->poseCmdService.empty())
          {
            ignerr << "Failed to create valid pose command service for world ["
                   << this->worldName <<"]" << std::endl;
            return;
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
      if (this->keyEvent.Control())
      {
        // Translate to world frame for snapping
        distance += this->startWorldPos;
        math::Vector3d snapVals = this->xyzSnap;

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
        math::Vector3d snapVals = this->rpySnap;

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
        math::Vector3d snapVals = this->scaleSnap;

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
rendering::NodePtr TransformControlLogicPrivate::TopLevelNode(
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
void TransformControlLogicPrivate::SetXYZSnap(const math::Vector3d &_xyz)
{
  this->xyzSnap = _xyz;
}

/////////////////////////////////////////////////
void TransformControlLogicPrivate::SetRPYSnap(const math::Vector3d &_rpy)
{
  this->rpySnap = _rpy;
}

/////////////////////////////////////////////////
void TransformControlLogicPrivate::SetScaleSnap(const math::Vector3d &_scale)
{
  this->scaleSnap = _scale;
}

/////////////////////////////////////////////////
double TransformControlLogicPrivate::SnapValue(
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
void TransformControlLogicPrivate::XYZConstraint(math::Vector3d &_axis)
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
void TransformControlLogicPrivate::SnapPoint(
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

bool TransformControlLogicPrivate::OnTransformMode(const msgs::StringMsg &_msg,
    msgs::Boolean &_res)
{
  std::string _mode = _msg.data();
  std::lock_guard<std::mutex> lock(this->mutex);
  if (_mode == "select")
    this->transformMode = rendering::TransformMode::TM_NONE;
  else if (_mode == "translate")
    this->transformMode = rendering::TransformMode::TM_TRANSLATION;
  else if (_mode == "rotate")
    this->transformMode = rendering::TransformMode::TM_ROTATION;
  else if (_mode == "scale")
    this->transformMode = rendering::TransformMode::TM_SCALE;
  else
    ignerr << "Unknown transform mode: [" << _mode << "]" << std::endl;

  ignition::gazebo::gui::events::TransformControlMode transformControlMode(
    this->transformMode);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &transformControlMode);

  this->HandleTransform();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
TransformControlLogic::TransformControlLogic()
  : GuiSystem(), dataPtr(new TransformControlLogicPrivate)
{
}

/////////////////////////////////////////////////
TransformControlLogic::~TransformControlLogic()
{
}

/////////////////////////////////////////////////
void TransformControlLogic::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Select entities";

  // transform mode
  this->dataPtr->transformModeService =
      "/gui/transform_mode";
  this->dataPtr->node.Advertise(this->dataPtr->transformModeService,
      &TransformControlLogicPrivate::OnTransformMode, this->dataPtr.get());
  ignmsg << "Transform mode service on ["
         << this->dataPtr->transformModeService << "]" << std::endl;

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void TransformControlLogic::Update(const UpdateInfo &/* _info */,
    EntityComponentManager &_ecm)
{
  if (this->dataPtr->worldName.empty())
  {
  Entity worldEntity;
  _ecm.Each<components::World, components::Name>(
      [&](const Entity &_entity,
        const components::World * /* _world */ ,
        const components::Name *_name)->bool
      {
        this->dataPtr->worldName = _name->Data();
        worldEntity = _entity;
        return true;
      });
  }
}

/////////////////////////////////////////////////
bool TransformControlLogic::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    ignition::gazebo::gui::events::EntitiesSelected *_e =
      static_cast<ignition::gazebo::gui::events::EntitiesSelected*>(_event);
    this->dataPtr->selectedEntities = _e->Data();
    this->dataPtr->HandleTransform();
  }
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    if (this->dataPtr->transformControl.Active())
      this->dataPtr->mouseDirty = true;
    this->dataPtr->HandleTransform();
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
    this->dataPtr->HandleTransform();
  }
  else if (_event->type() == ignition::gui::events::KeyPressOnScene::kType)
  {
    ignition::gui::events::KeyPressOnScene *_e =
      static_cast<ignition::gui::events::KeyPressOnScene*>(_event);
    this->dataPtr->keyEvent = _e->Key();
  }
  else if (_event->type() ==
      ignition::gui::events::SnapIntervals::kType)
  {
    auto snapEvent = reinterpret_cast<ignition::gui::events::SnapIntervals *>(
        _event);
    if (snapEvent)
    {
      this->dataPtr->SetXYZSnap(snapEvent->Position());
      this->dataPtr->SetRPYSnap(snapEvent->Rotation());
      this->dataPtr->SetScaleSnap(snapEvent->Scale());
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::plugins::TransformControlLogic,
                    ignition::gui::Plugin)
