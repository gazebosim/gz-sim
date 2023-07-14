/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <string>

#include <gz/common/Console.hh>
#include <gz/common/MouseEvent.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Utils.hh>
#include <gz/rendering/ArrowVisual.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Helpers.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Plane.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/PID.hh>
#include <gz/math/Inertial.hh>

#include "MouseDrag.hh"

namespace gz
{
namespace sim
{
  /// \enum MouseDragMode
  /// \brief Unique identifiers for mouse dragging modes
  enum MouseDragMode
  {
    /// \brief Inactive state
    NONE = 0,
    /// \brief Rotation mode
    ROTATE = 1,
    /// \brief Translation mode
    TRANSLATE = 2,
  };

  class MouseDragPrivate
  {
    /// \brief Handle mouse events
    public: void HandleMouseEvents();

    /// \brief Perform rendering calls in the rendering thread.
    public: void OnRender();

    /// \brief Update the PID loop on 3 coordinates and return the command value
    public: math::Vector3d CalculatePID(
      math::PID _pid[3],
      const math::Vector3d &_error,
      const std::chrono::duration<double> &_dt);

    /// \brief Update the PID gains with critical damping
    public: void UpdateGains(math::Pose3d _linkWorldPose,
      math::Inertiald _inertial);

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher for EntityWrench messages
    public: transport::Node::Publisher pub;

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery{nullptr};

    /// \brief Holds the latest mouse event
    public: gz::common::MouseEvent mouseEvent;

    /// \brief True if there are new mouse events to process
    public: bool mouseDirty{false};

    /// \brief True if there are new dragging mode changes to process
    public: bool modeDirty{false};

    /// \brief True if the force should be applied to the center of mass
    public: bool applyCOM{false};

    /// \brief Block orbit
    public: bool blockOrbit{false};

    /// \brief True if BlockOrbit events should be sent
    public: bool sendBlockOrbit{false};

    /// \brief Visual of the Link to which the wrenches are applied
    public: Entity visualId;

    /// \brief Application point of the wrench in world coordinates
    math::Vector3d applicationPoint;

    /// \brief Point to which the link is dragged to, in translation mode
    math::Vector3d target;

    /// \brief Offset of the force application point relative to the
    /// link origin, expressed in the link-fixed frame
    public: math::Vector3d offset{0.0, 0.0, 0.0};

    /// \brief Plane of force application
    public: math::Planed plane;

    /// \brief Initial position of a mouse click
    public: math::Vector2i mousePressPos;

    /// \brief Initial world rotation of the link during mouse click
    public: math::Quaterniond initialRot;

    /// \brief Current dragging mode
    public: MouseDragMode mode = MouseDragMode::NONE;

    /// \brief PID controllers for translation
    public: math::PID posPid[3];

    /// \brief PID controllers for rotation
    public: math::PID rotPid[3];

    /// \brief Spring stiffness for translation, in (m/s²)/m
    public: double posStiffness = 100;

    /// \brief Spring stiffness for rotation
    public: double rotStiffness = 100;

    /// \brief Arrow for visualizing wrench
    public: rendering::ArrowVisualPtr arrowVisual;
  };
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
MouseDrag::MouseDrag()
  : GuiSystem(), dataPtr(std::make_unique<MouseDragPrivate>())
{
}

/////////////////////////////////////////////////
MouseDrag::~MouseDrag() = default;

/////////////////////////////////////////////////
void MouseDrag::LoadConfig(const tinyxml2::XMLElement */*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "Mouse drag";

  // Create wrench publisher
  auto worldNames = gz::gui::worldNames();
  if (!worldNames.empty())
  {
    auto topic = transport::TopicUtils::AsValidTopic(
      "/world/" + worldNames[0].toStdString() + "/wrench");
    if (topic == "")
    {
      gzerr << "Unable to create publisher" << std::endl;
      return;
    }
    this->dataPtr->pub =
      this->dataPtr->node.Advertise<msgs::EntityWrench>(topic);
    gzdbg << "Created publisher to " << topic << std::endl;
  }

  gz::gui::App()->findChild<gz::gui::MainWindow *>
    ()->installEventFilter(this);
  gz::gui::App()->findChild<gz::gui::MainWindow *>
    ()->QuickWindow()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool MouseDrag::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  else if (_event->type() == gz::gui::events::LeftClickOnScene::kType)
  {
    auto event =
      static_cast<gz::gui::events::LeftClickOnScene *>(_event);
    this->dataPtr->mouseEvent = event->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == gz::gui::events::RightClickOnScene::kType)
  {
    auto event =
      static_cast<gz::gui::events::RightClickOnScene *>(_event);
    this->dataPtr->mouseEvent = event->Mouse();
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

  this->dataPtr->HandleMouseEvents();

  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void MouseDrag::Update(const UpdateInfo &_info,
  EntityComponentManager &_ecm)
{
  if (this->dataPtr->mode == MouseDragMode::NONE ||
      _info.paused)
  {
    this->dataPtr->mode = MouseDragMode::NONE;
    return;
  }

  // Get Link corresponding to clicked Visual
  Link link;
  auto linkId =
    _ecm.ComponentData<components::ParentEntity>(this->dataPtr->visualId);
  if (linkId)
  {
    link = Link(*linkId);
    if (!link.Valid(_ecm))
    {
      return;
    }
  }
  link.EnableVelocityChecks(_ecm, true);

  auto linkWorldPose = worldPose(*linkId, _ecm);
  auto inertial = _ecm.Component<components::Inertial>(*linkId);
  if (!inertial)
  {
    return;
  }
  this->dataPtr->UpdateGains(linkWorldPose, inertial->Data());

  if (this->dataPtr->modeDirty)
  {
    this->dataPtr->modeDirty = false;

    if (this->dataPtr->mode == MouseDragMode::ROTATE)
    {
      this->dataPtr->mousePressPos = this->dataPtr->mouseEvent.Pos();
      this->dataPtr->initialRot = linkWorldPose.Rot();
    }
    else if (this->dataPtr->mode == MouseDragMode::TRANSLATE)
    {
      // Calculate offset of force application from link origin
      if (this->dataPtr->applyCOM)
      {
        this->dataPtr->offset = inertial->Data().Pose().Pos();
        this->dataPtr->applicationPoint =
          linkWorldPose.Pos() +
          linkWorldPose.Rot().RotateVector(this->dataPtr->offset);
      }
      else
      {
        this->dataPtr->offset = linkWorldPose.Rot().RotateVectorReverse(
          this->dataPtr->applicationPoint - linkWorldPose.Pos());
      }
    }

    // The plane of wrench application should be normal to the center
    // of the camera view and pass through the application point
    math::Vector3d direction = this->dataPtr->camera->WorldPose().Rot().XAxis();
    double planeOffset = direction.Dot(this->dataPtr->applicationPoint);
    this->dataPtr->plane = math::Planed(direction, planeOffset);

    gzdbg << "Resetting PIDs" << std::endl;
    for (auto i : {0, 1, 2})
    {
      this->dataPtr->posPid[i].Reset();
      this->dataPtr->rotPid[i].Reset();
    }
  }
  // Track the application point in translation mode
  else if (this->dataPtr->mode == MouseDragMode::TRANSLATE)
  {
    this->dataPtr->applicationPoint =
      linkWorldPose.Pos() +
      linkWorldPose.Rot().RotateVector(this->dataPtr->offset);
  }

  // Normalize mouse position on the image
  double width = this->dataPtr->camera->ImageWidth();
  double height = this->dataPtr->camera->ImageHeight();

  double nx = 2.0 * this->dataPtr->mouseEvent.Pos().X() / width - 1.0;
  double ny = 1.0 - 2.0 * this->dataPtr->mouseEvent.Pos().Y() / height;

  // Make a ray query at the mouse position
  this->dataPtr->rayQuery->SetFromCamera(
    this->dataPtr->camera, math::Vector2d(nx, ny));

  math::Vector3d origin = this->dataPtr->rayQuery->Origin();
  math::Vector3d direction = this->dataPtr->rayQuery->Direction();

  if (auto t = this->dataPtr->plane.Intersection(origin, direction))
    this->dataPtr->target = *t;
  else
    return;

  math::Vector3d force;
  math::Vector3d torque;
  math::Quaterniond errorRot;
  math::Vector3d errorPos;

  if (this->dataPtr->mode == MouseDragMode::ROTATE)
  {
    // Calculate rotation angle from mouse displacement
    math::Vector3d camPos = this->dataPtr->camera->WorldPose().Pos();
    math::Vector3d start =
      (this->dataPtr->applicationPoint - camPos).Normalized();
    math::Vector3d end = (this->dataPtr->target - camPos).Normalized();
    math::Vector3d axis = start.Cross(end);
    double angle = -atan2(axis.Length(), start.Dot(end));

    // Project rotation axis onto plane
    axis -= axis.Dot(this->dataPtr->plane.Normal()) *
      this->dataPtr->plane.Normal();
    axis.Normalize();

    // Calculate the necessary rotation and torque
    errorRot =
      (this->dataPtr->initialRot * math::Quaterniond(axis, angle)).Inverse() *
      linkWorldPose.Rot();
    torque = this->dataPtr->CalculatePID(
      this->dataPtr->rotPid, errorRot.Euler(), _info.dt);
  }
  else if (this->dataPtr->mode == MouseDragMode::TRANSLATE)
  {
    errorPos = this->dataPtr->applicationPoint - this->dataPtr->target;
    force = this->dataPtr->CalculatePID(
      this->dataPtr->posPid, errorPos, _info.dt);
    torque =
      linkWorldPose.Rot().RotateVector(this->dataPtr->offset).Cross(force) +
      this->dataPtr->CalculatePID(
        this->dataPtr->rotPid, linkWorldPose.Rot().Euler(), _info.dt);
  }

  // Publish wrench
  msgs::EntityWrench msg;
  msg.mutable_entity()->set_id(*linkId);
  msgs::Set(msg.mutable_wrench()->mutable_force(), force);
  msgs::Set(msg.mutable_wrench()->mutable_torque(), torque);

  this->dataPtr->pub.Publish(msg);
}

/////////////////////////////////////////////////
void MouseDrag::OnSwitchCOM(const bool _checked)
{
  this->dataPtr->applyCOM = _checked;
}

/////////////////////////////////////////////////
void MouseDragPrivate::OnRender()
{
  // Get scene and user camera
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (!this->scene)
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
        gzdbg << "MouseDrag plugin is using camera ["
              << this->camera->Name() << "]" << std::endl;
        break;
      }
    }

    if (!this->camera)
    {
      gzerr << "MouseDrag camera is not available" << std::endl;
      return;
    }
    this->rayQuery = this->scene->CreateRayQuery();
    this->arrowVisual = this->scene->CreateArrowVisual();
    this->arrowVisual->SetMaterial("Default/TransYellow");
  }

  // Update the visualization
  if (this->mode == MouseDragMode::NONE)
  {
    this->arrowVisual->ShowArrowHead(false);
    this->arrowVisual->ShowArrowShaft(false);
    this->arrowVisual->ShowArrowRotation(false);
  }
  else
  {
    math::Vector3d axisDir = this->target - this->applicationPoint;
    math::Vector3d u = axisDir.Normalized();
    math::Vector3d v = gz::math::Vector3d::UnitZ;
    double angle = acos(v.Dot(u));
    math::Quaterniond quat;
    // check the parallel case
    if (math::equal(angle, GZ_PI))
      quat.SetFromAxisAngle(u.Perpendicular(), angle);
    else
      quat.SetFromAxisAngle((v.Cross(u)).Normalize(), angle);

    if (this->mode == MouseDragMode::ROTATE)
    {
      quat =
        math::Quaterniond(this->camera->WorldPose().Rot().XAxis(), GZ_PI/2) *
        quat;
      double scale = axisDir.Length() / 0.075f;

      this->arrowVisual->SetLocalPosition(this->applicationPoint);
      this->arrowVisual->SetLocalRotation(quat);
      this->arrowVisual->SetLocalScale(scale, scale, 0);

      this->arrowVisual->ShowArrowHead(false);
      this->arrowVisual->ShowArrowShaft(false);
      this->arrowVisual->ShowArrowRotation(true);
    }
    if (this->mode == MouseDragMode::TRANSLATE)
    {
      this->arrowVisual->SetLocalPosition(this->applicationPoint);
      this->arrowVisual->SetLocalRotation(quat);
      this->arrowVisual->SetLocalScale(1, 1, axisDir.Length());

      this->arrowVisual->ShowArrowHead(true);
      this->arrowVisual->ShowArrowShaft(true);
      this->arrowVisual->ShowArrowRotation(false);
    }
  }

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
void MouseDragPrivate::HandleMouseEvents()
{
  // Check for mouse events
  if (!this->mouseDirty)
  {
    return;
  }
  this->mouseDirty = false;

  if (this->mouseEvent.Type() == common::MouseEvent::PRESS &&
      this->mouseEvent.Control() &&
      this->mouseEvent.Button() != common::MouseEvent::MIDDLE)
  {
    this->blockOrbit = true;
    this->sendBlockOrbit = true;
    this->modeDirty = true;

    gui::events::DeselectAllEntities event(true);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
        &event);

    // Get the visual at mouse position
    rendering::VisualPtr visual = this->scene->VisualAt(
      this->camera,
      this->mouseEvent.Pos());

    if (!visual)
    {
      this->mode = MouseDragMode::NONE;
      return;
    }
    try
    {
      this->visualId = std::get<uint64_t>(visual->UserData("gazebo-entity"));
    }
    catch(std::bad_variant_access &e)
    {
      this->mode = MouseDragMode::NONE;
      return;
    }

    // Get the 3D coordinates of the clicked point
    this->applicationPoint = rendering::screenToScene(
      this->mouseEvent.Pos(), this->camera,
      this->rayQuery);

    if (this->mouseEvent.Button() == common::MouseEvent::LEFT)
    {
      this->mode = MouseDragMode::ROTATE;
    }
    else if (this->mouseEvent.Button() == common::MouseEvent::RIGHT)
    {
      this->mode = MouseDragMode::TRANSLATE;
    }
  }
  else if (this->mouseEvent.Type() == common::MouseEvent::RELEASE)
  {
    this->mode = MouseDragMode::NONE;
    this->blockOrbit = false;
    this->sendBlockOrbit = true;
  }
  else if (this->mouseEvent.Type() == common::MouseEvent::MOVE)
  {
    if (this->mode != MouseDragMode::NONE)
    {
      this->blockOrbit = true;
      this->sendBlockOrbit = true;
    }
  }
}

/////////////////////////////////////////////////
math::Vector3d MouseDragPrivate::CalculatePID(
  math::PID _pid[3],
  const math::Vector3d &_error,
  const std::chrono::duration<double> &_dt)
{
  math::Vector3d result;
  for (auto i : {0, 1, 2})
  {
    result[i] = _pid[i].Update(_error[i], _dt);
  }
  return result;
}

/////////////////////////////////////////////////
void MouseDragPrivate::UpdateGains(math::Pose3d _linkWorldPose,
  math::Inertiald _inertial)
{
  math::Matrix3d R(_linkWorldPose.Rot() * _inertial.Pose().Rot());
  math::Matrix3d inertia = R * _inertial.Moi() * R.Transposed();
  if (this->mode == MouseDragMode::ROTATE)
  {
    for (auto i : {0, 1, 2})
    {
      this->rotPid[i].SetPGain(this->rotStiffness * inertia(i, i));
      this->rotPid[i].SetDGain(2 * sqrt(this->rotStiffness) * inertia(i, i));
    }
  }
  else if (this->mode == MouseDragMode::TRANSLATE)
  {
    double mass = _inertial.MassMatrix().Mass();
    for (auto i : {0, 1, 2})
    {
      this->posPid[i].SetPGain(this->posStiffness * mass);
      this->posPid[i].SetDGain(2 * sqrt(this->posStiffness) * mass);
      this->rotPid[i].SetPGain(0);
      this->rotPid[i].SetDGain(sqrt(this->rotStiffness) * inertia(i, i));
    }
  }
}

// Register this plugin
GZ_ADD_PLUGIN(MouseDrag, gz::gui::Plugin);
