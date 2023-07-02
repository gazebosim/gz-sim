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
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Helpers.hh>
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
#include <gz/math/Vector3.hh>
#include <gz/math/Plane.hh>

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
    public: bool applyCOM{true};

    /// \brief Block orbit
    public: bool blockOrbit{false};

    /// \brief Visual of the Link to which apply the wrenches
    public: Entity visualId;

    /// \brief Offset of the force application point relative to the
    /// center of mass
    public: math::Vector3d offset{0.0, 0.0, 0.0};

    /// \brief Plane of force application
    public: math::Planed plane;

    /// \brief Initial position of a mouse click
    public: math::Vector2i mousePressPos = math::Vector2i::Zero;

    /// \brief Current dragging mode
    public: MouseDragMode mode = MouseDragMode::NONE;
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
void MouseDrag::Update(const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  if (this->dataPtr->mode == MouseDragMode::NONE ||
      this->dataPtr->visualId == kNullEntity)
  {
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

  auto inertial = _ecm.Component<components::Inertial>(*linkId);
  math::Pose3d linkWorldPose = worldPose(*linkId, _ecm);
  if (!inertial)
  {
    gzdbg << "Link must have an inertial component" << std::endl;
    return;
  }
  auto centerOfMass =
    linkWorldPose.Pos() +
    linkWorldPose.Rot().RotateVector(inertial->Data().Pose().Pos());
  math::Vector3d applicationPoint = centerOfMass;
  if (!this->dataPtr->applyCOM)
  {
    applicationPoint +=
      linkWorldPose.Rot().RotateVector(this->dataPtr->offset);
  }

  // Recalculate the plane of application when necessary
  if (this->dataPtr->modeDirty)
  {
    this->dataPtr->modeDirty = false;

    // Make a ray query at the center of the image
    this->dataPtr->rayQuery->SetFromCamera(
      this->dataPtr->camera, math::Vector2d(0, 0));

    math::Vector3d direction = this->dataPtr->rayQuery->Direction();

    double planeOffset = direction.AbsDot(applicationPoint);
    this->dataPtr->plane = math::Planed(-direction, planeOffset);

    gzdbg << "Mode " << this->dataPtr->mode
          << " Entity " << *linkId
          << " Point " << applicationPoint
          << " Normal " << -direction << std::endl;
  }

  math::Vector3d force;
  math::Vector3d torque;

  if (this->dataPtr->mode == MouseDragMode::ROTATE)
  {
    // TODO(anyone): implement
    torque = math::Vector3d::Zero;
  }
  else if (this->dataPtr->mode == MouseDragMode::TRANSLATE)
  {
    // Normalize point on the image
    double width = this->dataPtr->camera->ImageWidth();
    double height = this->dataPtr->camera->ImageHeight();

    double nx = 2.0 * this->dataPtr->mouseEvent.Pos().X() / width - 1.0;
    double ny = 1.0 - 2.0 * this->dataPtr->mouseEvent.Pos().Y() / height;

    // Make a ray query at the mouse position
    this->dataPtr->rayQuery->SetFromCamera(
      this->dataPtr->camera, math::Vector2d(nx, ny));

    math::Vector3d origin = this->dataPtr->rayQuery->Origin();
    math::Vector3d direction = this->dataPtr->rayQuery->Direction();

    auto target = this->dataPtr->plane.Intersection(origin, direction);
    if (!target)
    {
      gzdbg << "No intersection found" << std::endl;
      return;
    }
    math::Vector3d displacement = *target - applicationPoint;
    // gzdbg << "Displacement " << displacement << std::endl;

    static math::Vector3d displacementPrev;

    force = 1e3*displacement + 1e4*(displacement-displacementPrev);
    displacementPrev = displacement;
  }

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
    this->rayQuery = this->camera->Scene()->CreateRayQuery();
  }

  gz::gui::events::BlockOrbit blockOrbitEvent(this->blockOrbit);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &blockOrbitEvent);
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
      this->mouseEvent.Control())
  {
    this->mousePressPos = this->mouseEvent.Pos();

    // Get the visual at mouse position
    rendering::VisualPtr visual = this->scene->VisualAt(
      this->camera,
      this->mouseEvent.Pos());
    this->visualId = kNullEntity;

    if (!visual)
    {
      return;
    }

    try
    {
      this->visualId = std::get<uint64_t>(visual->UserData("gazebo-entity"));
    }
    catch(std::bad_variant_access &e)
    {
      // It's ok to get here
    }

    if (this->mouseEvent.Button() == common::MouseEvent::LEFT)
    {
      // TODO(anyone): revert to rotation mode
      // this->mode = MouseDragMode::ROTATE;
      this->mode = MouseDragMode::TRANSLATE;
      this->modeDirty = true;
    }
    else if (this->mouseEvent.Button() == common::MouseEvent::RIGHT)
    {
      this->mode = MouseDragMode::TRANSLATE;
      this->modeDirty = true;
    }
  }
  else if (this->mouseEvent.Type() == common::MouseEvent::RELEASE)
  {
    this->mode = MouseDragMode::NONE;
    this->blockOrbit = false;
    this->visualId = kNullEntity;
    gzdbg << "Released" << std::endl;
  }
  else if (this->mouseEvent.Type() == common::MouseEvent::MOVE)
  {
    if (this->mode != MouseDragMode::NONE)
    {
      this->blockOrbit = true;
    }
  }
}

// Register this plugin
GZ_ADD_PLUGIN(MouseDrag, gz::gui::Plugin);
