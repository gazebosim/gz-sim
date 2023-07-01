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
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Helpers.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

#include "MouseDrag.hh"

namespace gz
{
namespace sim
{
  class MouseDragPrivate
  {
    /// \brief Handle mouse events
    public: void HandleMouseEvents();

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher for EntityWrench messages
    public: transport::Node::Publisher pub;

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Holds the latest mouse event
    public: gz::common::MouseEvent mouseEvent;

    /// \brief True if there are new mouse events to process.
    public: bool mouseDirty{false};

    /// \brief True if the force should be applied to the center of mass
    public: bool applyCOM{true};

    /// \brief Block orbit
    public: bool blockOrbit{false};

    /// \brief Visual of the Link to which apply the wrenches
    public: Entity visualId;

    /// \brief Offset of the force application point relative to the
    /// center of mass
    public: math::Vector3d offset{0.0, 0.0, 0.0};
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
    gz::gui::events::BlockOrbit blockOrbitEvent(this->dataPtr->blockOrbit);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
        &blockOrbitEvent);
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
  if (this->dataPtr->visualId == kNullEntity)
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
  if (!inertial)
  {
    gzdbg << "Link must have an inertial component" << std::endl;
    return;
  }
  auto centerOfMass = inertial->Data().Pose().Pos();
}

/////////////////////////////////////////////////
void MouseDrag::OnSwitchCOM(const bool /* _checked */)
{
  // this->dataPtr->applyCOM = _checked;
  gzdbg << "Only CoM application is currently supported"
        << this->dataPtr->applyCOM << std::endl;
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

  // Get scene and user camera
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
        gzdbg << "MouseDrag plugin is using camera ["
              << this->camera->Name() << "]" << std::endl;
        break;
      }
    }
  }

  if (this->mouseEvent.Type() == common::MouseEvent::PRESS &&
      this->mouseEvent.Control())
  {
    this->blockOrbit = true;

    // Get the visual at mouse position
    rendering::VisualPtr visual = this->scene->VisualAt(
      this->camera,
      this->mouseEvent.Pos());

    this->visualId = kNullEntity;
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
      gzdbg << "Ctrl-Left click press" << std::endl;
    }
    else if (this->mouseEvent.Button() == common::MouseEvent::RIGHT)
    {
      gzdbg << "Ctrl-Right click press" << std::endl;
    }
  }
  else if (this->mouseEvent.Type() == common::MouseEvent::RELEASE)
  {
    this->blockOrbit = false;
    this->visualId = kNullEntity;
    gzdbg << "Click release" << std::endl;
  }
  else if (this->mouseEvent.Type() == common::MouseEvent::MOVE)
  {
    gzdbg << "Mouse Move [" << this->mouseEvent.Pos() << "]" << std::endl;
  }
}

// Register this plugin
GZ_ADD_PLUGIN(MouseDrag, gz::gui::Plugin);
