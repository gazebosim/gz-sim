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

#include <memory>
#include <mutex>
#include <string>

#include <gz/common/MouseEvent.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/ArrowVisual.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/GizmoVisual.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RenderTypes.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/TransformType.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/rendering/WrenchVisualizer.hh>
#include <gz/transport/Node.hh>

#include "ApplyForceTorque.hh"

namespace gz
{
namespace sim
{
  /// \enum RotationToolVector
  /// \brief Unique identifiers for which vector is currently being
  /// modified by the rotation tool
  enum RotationToolVector
  {
    /// \brief Rotation tool is inactive
    NONE = 0,
    /// \brief Force vector
    FORCE = 1,
    /// \brief Torque vector
    TORQUE = 2,
  };

  class ApplyForceTorquePrivate
  {
    /// \brief Publish EntityWrench messages in order to apply force and torque
    /// \param[in] _applyForce True if the force should be applied
    /// \param[in] _applyTorque True if the torque should be applied
    public: void PublishWrench(bool _applyForce, bool _applyTorque);

    /// \brief Perform rendering calls in the rendering thread.
    public: void OnRender();

    /// \brief Update visuals for force and torque
    public: void UpdateVisuals();

    /// \brief Handle mouse events
    public: void HandleMouseEvents();

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher for EntityWrench messages
    public: transport::Node::Publisher pub;

    /// \brief World name
    public: std::string worldName;

    /// \brief True if the ApplyLinkWrench system is loaded
    public: bool systemLoaded{false};

    /// \brief To synchronize member access
    public: std::mutex mutex;

    /// \brief Name of the selected model
    public: QString modelName;

    /// \brief List of the name of the links in the selected model
    public: QStringList linkNameList;

    /// \brief Index of selected link in list
    public: int linkIndex{-1};

    /// \brief Entity of the currently selected Link
    public: std::optional<Entity> selectedEntity;

    /// \brief True if a new entity was selected
    public: bool changedEntity{false};

    /// \brief True if a new link was selected from the dropdown
    public: bool changedIndex{false};

    /// \brief Force to be applied in link-fixed frame
    public: math::Vector3d force{0.0, 0.0, 0.0};

    /// \brief Offset of force application point in link-fixed frame
    /// relative to the center of mass
    public: math::Vector3d offset{0.0, 0.0, 0.0};

    /// \brief Torque to be applied in link-fixed frame
    public: math::Vector3d torque{0.0, 0.0, 0.0};

    /// \brief Pose of the link-fixed frame
    public: math::Pose3d linkWorldPose;

    /// \brief Pose of the inertial frame relative to the link frame
    public: math::Pose3d inertialPose;

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Ray used for checking intersection with planes for computing
    /// 3d world coordinates from 2d
    public: rendering::RayQueryPtr ray{nullptr};

    /// \brief True if there are new mouse events to process.
    public: bool mouseDirty{false};

    /// \brief True if the rotation tool modified the force or torque vector
    public: bool vectorDirty{false};

    /// \brief Whether the transform gizmo is being dragged.
    public: bool transformActive{false};

    /// \brief Block orbit
    public: bool blockOrbit{false};

    /// \brief True if BlockOrbit events should be sent
    public: bool sendBlockOrbit{false};

    /// \brief Where the mouse left off
    public: math::Vector2i mousePressPos = math::Vector2i::Zero;

    /// \brief Holds the latest mouse event
    public: gz::common::MouseEvent mouseEvent;

    /// \brief Vector (force or torque) currently being rotated
    /// by the rotation tool
    public: RotationToolVector activeVector{RotationToolVector::NONE};

    /// \brief Vector value on start of rotation tool transformation,
    /// relative to link
    public: math::Vector3d initialVector;

    /// \brief Vector orientation on start of rotation tool transformation,
    /// relative to link
    public: math::Quaterniond initialVectorRot;

    /// \brief Current orientation of the transformed vector relative to link
    public: math::Quaterniond vectorRot;

    /// \brief Active transformation axis on the rotation tool
    public: rendering::TransformAxis activeAxis{rendering::TA_NONE};

    /// \brief Wrench visualizer
    public: detail::WrenchVisualizer wrenchVis;

    /// \brief Arrow for visualizing force.
    public: rendering::ArrowVisualPtr forceVisual{nullptr};

    /// \brief Arrow for visualizing torque.
    public: rendering::VisualPtr torqueVisual{nullptr};

    /// \brief Gizmo visual for rotating vectors in rotation tool
    public: rendering::GizmoVisualPtr gizmoVisual{nullptr};
  };
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
ApplyForceTorque::ApplyForceTorque()
  : GuiSystem(), dataPtr(std::make_unique<ApplyForceTorquePrivate>())
{
}

/////////////////////////////////////////////////
ApplyForceTorque::~ApplyForceTorque()
{
  if (!this->dataPtr->scene)
    return;
  this->dataPtr->scene->DestroyNode(this->dataPtr->forceVisual, true);
  this->dataPtr->scene->DestroyNode(this->dataPtr->torqueVisual, true);
  this->dataPtr->scene->DestroyNode(this->dataPtr->gizmoVisual, true);
}

/////////////////////////////////////////////////
void ApplyForceTorque::LoadConfig(const tinyxml2::XMLElement */*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "Apply force and torque";

  // Create wrench publisher
  auto worldNames = gz::gui::worldNames();
  if (!worldNames.empty())
  {
    this->dataPtr->worldName = worldNames[0].toStdString();
    auto topic = transport::TopicUtils::AsValidTopic(
      "/world/" + this->dataPtr->worldName + "/wrench");
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
}

/////////////////////////////////////////////////
bool ApplyForceTorque::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();

    if (this->dataPtr->vectorDirty)
    {
      this->dataPtr->vectorDirty = false;
      if (this->dataPtr->activeVector == RotationToolVector::FORCE)
        emit this->ForceChanged();
      else if (this->dataPtr->activeVector == RotationToolVector::TORQUE)
        emit this->TorqueChanged();
    }
  }
  else if (_event->type() == gz::sim::gui::events::EntitiesSelected::kType)
  {
    if (!this->dataPtr->blockOrbit && !this->dataPtr->mouseEvent.Dragging())
    {
      gz::sim::gui::events::EntitiesSelected *_e =
          static_cast<gz::sim::gui::events::EntitiesSelected*>(_event);
      this->dataPtr->selectedEntity = _e->Data().front();
      this->dataPtr->changedEntity = true;
    }
  }
  else if (_event->type() == gz::sim::gui::events::DeselectAllEntities::kType)
  {
    if (!this->dataPtr->blockOrbit && !this->dataPtr->mouseEvent.Dragging())
    {
      this->dataPtr->selectedEntity.reset();
      this->dataPtr->changedEntity = true;
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

  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ApplyForceTorque::Update(const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Load the ApplyLinkWrench system
  // TODO(anyone): should this be checked on every Update instead?
  if (!this->dataPtr->systemLoaded)
  {
    const std::string name{"gz::sim::systems::ApplyLinkWrench"};
    const std::string filename{"gz-sim-apply-link-wrench-system"};
    const std::string innerxml{"<verbose>0</verbose>"};

    // Get world entity
    Entity worldEntity;
    _ecm.Each<components::World, components::Name>(
      [&](const Entity &_entity,
        const components::World */*_world*/,
        const components::Name *_name)->bool
      {
        if (_name->Data() == this->dataPtr->worldName)
        {
          worldEntity = _entity;
          return false;
        }
        return true;
      });

    // Check if already loaded
    auto msg = _ecm.ComponentData<components::SystemPluginInfo>(worldEntity);
    if (!msg)
    {
      gzdbg << "Unable to find SystemPluginInfo component for entity "
            << worldEntity << std::endl;
      return;
    }
    for (const auto &plugin : msg->plugins())
    {
      if (plugin.filename() == filename)
      {
        this->dataPtr->systemLoaded = true;
        gzdbg << "ApplyLinkWrench system already loaded" << std::endl;
        break;
      }
    }

    // Request to load system
    if (!this->dataPtr->systemLoaded)
    {
      msgs::EntityPlugin_V req;
      req.mutable_entity()->set_id(worldEntity);
      auto plugin = req.add_plugins();
      plugin->set_name(name);
      plugin->set_filename(filename);
      plugin->set_innerxml(innerxml);

      msgs::Boolean res;
      bool result;
      unsigned int timeout = 5000;
      std::string service{"/world/" + this->dataPtr->worldName +
          "/entity/system/add"};
      if (this->dataPtr->node.Request(service, req, timeout, res, result))
      {
        this->dataPtr->systemLoaded = true;
        gzdbg << "ApplyLinkWrench system has been loaded" << std::endl;
      }
      else
      {
        gzerr << "Error adding new system to entity: "
              << worldEntity << "\n"
              << "Name: " << name << "\n"
              << "Filename: " << filename << "\n"
              << "Inner XML: " << innerxml << std::endl;
      }
    }
  }

  if (this->dataPtr->changedEntity)
  {
    this->dataPtr->changedEntity = false;

    this->dataPtr->modelName = "";
    this->dataPtr->linkNameList.clear();
    this->dataPtr->linkIndex = -1;

    if (this->dataPtr->selectedEntity.has_value())
    {
      Model parentModel(*this->dataPtr->selectedEntity);
      Link selectedLink(*this->dataPtr->selectedEntity);
      if (parentModel.Valid(_ecm))
      {
        selectedLink = Link(parentModel.CanonicalLink(_ecm));
      }
      else if (selectedLink.Valid(_ecm))
      {
        parentModel = *selectedLink.ParentModel(_ecm);
      }
      else
      {
        this->dataPtr->selectedEntity.reset();
        return;
      }

      this->dataPtr->modelName = QString::fromStdString(
        parentModel.Name(_ecm));
      this->dataPtr->selectedEntity = selectedLink.Entity();

      // Put all of the model's links into the list
      auto links = parentModel.Links(_ecm);
      unsigned int i{0};
      while (i < links.size())
      {
        Link link(links[i]);
        this->dataPtr->linkNameList.push_back(
          QString::fromStdString(*link.Name(_ecm)));
        if (link.Entity() == this->dataPtr->selectedEntity)
        {
          this->dataPtr->linkIndex = i;
        }
        ++i;
      }
    }
  }

  if (this->dataPtr->changedIndex)
  {
    this->dataPtr->changedIndex = false;

    if (this->dataPtr->selectedEntity.has_value())
    {
      auto parentModel = Link(*this->dataPtr->selectedEntity).ParentModel(_ecm);
      std::string linkName =
        this->dataPtr->linkNameList[this->dataPtr->linkIndex].toStdString();
      this->dataPtr->selectedEntity = parentModel->LinkByName(_ecm, linkName);
    }
  }

  // Get the position of the center of mass and link orientation
  if (this->dataPtr->selectedEntity.has_value())
  {
    auto linkWorldPose = worldPose(*this->dataPtr->selectedEntity, _ecm);
    auto inertial = _ecm.Component<components::Inertial>(
      *this->dataPtr->selectedEntity);
    if (inertial)
    {
      this->dataPtr->linkWorldPose = linkWorldPose;
      this->dataPtr->inertialPose = inertial->Data().Pose();
    }
  }

  emit this->ModelNameChanged();
  emit this->LinkNameListChanged();
  emit this->LinkIndexChanged();
}

/////////////////////////////////////////////////
QString ApplyForceTorque::ModelName() const
{
  return this->dataPtr->modelName;
}

/////////////////////////////////////////////////
QStringList ApplyForceTorque::LinkNameList() const
{
  return this->dataPtr->linkNameList;
}

/////////////////////////////////////////////////
int ApplyForceTorque::LinkIndex() const
{
  return this->dataPtr->linkIndex;
}

/////////////////////////////////////////////////
void ApplyForceTorque::SetLinkIndex(int _linkIndex)
{
  this->dataPtr->linkIndex = _linkIndex;
  this->dataPtr->changedIndex = true;
}

/////////////////////////////////////////////////
QVector3D ApplyForceTorque::Force() const
{
  return QVector3D(
    this->dataPtr->force.X(),
    this->dataPtr->force.Y(),
    this->dataPtr->force.Z());
}

/////////////////////////////////////////////////
void ApplyForceTorque::SetForce(QVector3D _force)
{
  this->dataPtr->force.Set(_force.x(), _force.y(), _force.z());
  // Update rotation tool orientation when force is set by components
  if (this->dataPtr->activeVector == RotationToolVector::FORCE
    && this->dataPtr->activeAxis == rendering::TransformAxis::TA_NONE)
  {
    this->dataPtr->vectorRot = math::Matrix4d::LookAt(
      -this->dataPtr->force, math::Vector3d::Zero).Rotation();
  }
  emit this->ForceMagChanged();
}

/////////////////////////////////////////////////
double ApplyForceTorque::ForceMag() const
{
  return this->dataPtr->force.Length();
}

/////////////////////////////////////////////////
void ApplyForceTorque::SetForceMag(double _forceMag)
{
  if (this->dataPtr->force == math::Vector3d::Zero)
  {
    this->dataPtr->force.X() = _forceMag;
  }
  else
  {
    this->dataPtr->force = _forceMag * this->dataPtr->force.Normalized();
  }
  emit this->ForceChanged();
}

/////////////////////////////////////////////////
QVector3D ApplyForceTorque::Torque() const
{
  return QVector3D(
    this->dataPtr->torque.X(),
    this->dataPtr->torque.Y(),
    this->dataPtr->torque.Z());
}

/////////////////////////////////////////////////
void ApplyForceTorque::SetTorque(QVector3D _torque)
{
  this->dataPtr->torque.Set(_torque.x(), _torque.y(), _torque.z());
  // Update rotation tool orientation when torque is set by components
  if (this->dataPtr->activeVector == RotationToolVector::TORQUE
    && this->dataPtr->activeAxis == rendering::TransformAxis::TA_NONE)
  {
    this->dataPtr->vectorRot = math::Matrix4d::LookAt(
      -this->dataPtr->torque, math::Vector3d::Zero).Rotation();
  }
  emit this->TorqueMagChanged();
}

/////////////////////////////////////////////////
double ApplyForceTorque::TorqueMag() const
{
  return this->dataPtr->torque.Length();
}

/////////////////////////////////////////////////
void ApplyForceTorque::SetTorqueMag(double _torqueMag)
{
  if (this->dataPtr->torque == math::Vector3d::Zero)
  {
    this->dataPtr->torque.X() = _torqueMag;
  }
  else
  {
    this->dataPtr->torque = _torqueMag * this->dataPtr->torque.Normalized();
  }
  emit this->TorqueChanged();
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateOffset(double _x, double _y, double _z)
{
  this->dataPtr->offset.Set(_x, _y, _z);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyForce()
{
  this->dataPtr->PublishWrench(true, false);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyTorque()
{
  this->dataPtr->PublishWrench(false, true);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyAll()
{
  this->dataPtr->PublishWrench(true, true);
}

/////////////////////////////////////////////////
void ApplyForceTorquePrivate::PublishWrench(bool _applyForce, bool _applyTorque)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (!this->selectedEntity.has_value())
  {
    gzdbg << "No link selected" << std::endl;
    return;
  }

  // Force and torque in world coordinates
  math::Vector3d forceToApply = _applyForce ?
    this->linkWorldPose.Rot().RotateVector(this->force) :
    math::Vector3d::Zero;
  math::Vector3d torqueToApply = _applyTorque ?
    this->linkWorldPose.Rot().RotateVector(this->torque) :
    math::Vector3d::Zero;
  // The ApplyLinkWrench system takes the offset in the link frame
  math::Vector3d offsetToApply = _applyForce ?
    this->offset + this->inertialPose.Pos() :
    math::Vector3d::Zero;

  msgs::EntityWrench msg;
  msg.mutable_entity()->set_id(*this->selectedEntity);
  msgs::Set(msg.mutable_wrench()->mutable_force(), forceToApply);
  msgs::Set(msg.mutable_wrench()->mutable_force_offset(), offsetToApply);
  msgs::Set(msg.mutable_wrench()->mutable_torque(), torqueToApply);

  this->pub.Publish(msg);
}

/////////////////////////////////////////////////
void ApplyForceTorquePrivate::OnRender()
{
  if (!this->scene)
  {
    // Get scene and user camera
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (!this->scene)
    {
      return;
    }

    for (unsigned int i = 0; i < this->scene->SensorCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->SensorByIndex(i));
      if (cam && cam->HasUserData("user-camera") &&
          std::get<bool>(cam->UserData("user-camera")))
      {
        this->camera = cam;
        gzdbg << "ApplyForceTorque plugin is using camera ["
              << this->camera->Name() << "]" << std::endl;
        break;
      }
    }

    this->ray = this->scene->CreateRayQuery();

    // Make material into overlay
    auto mat = this->scene->Material("Default/TransRed")->Clone();
    mat->SetDepthCheckEnabled(false);
    mat->SetDepthWriteEnabled(false);

    // Create visuals
    if (!this->wrenchVis.Init(this->scene))
    {
      gzerr << "Invalid scene" << std::endl;
      return;
    }
    this->forceVisual = this->wrenchVis.CreateForceVisual(mat);
    this->torqueVisual = this->wrenchVis.CreateTorqueVisual(mat);
    this->gizmoVisual = this->scene->CreateGizmoVisual();
    this->scene->RootVisual()->AddChild(this->gizmoVisual);
  }

  this->HandleMouseEvents();

  this->UpdateVisuals();

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
void ApplyForceTorquePrivate::UpdateVisuals()
{
  math::Pose3d inertialWorldPose = this->linkWorldPose * this->inertialPose;
  // Update force visualization
  if (this->force != math::Vector3d::Zero &&
      this->selectedEntity.has_value())
  {
    math::Vector3d worldForce =
      this->linkWorldPose.Rot().RotateVector(this->force);
    math::Vector3d applicationPoint =
      inertialWorldPose.Pos() +
      this->linkWorldPose.Rot().RotateVector(this->offset);
    double scale = applicationPoint.Distance(this->camera->WorldPose().Pos())
      / 2.0;
    this->wrenchVis.UpdateVectorVisual(
      this->forceVisual, worldForce, applicationPoint, scale, true);
    this->forceVisual->SetVisible(true);
  }
  else
  {
    this->forceVisual->SetVisible(false);
  }

  // Update torque visualization
  if (this->torque != math::Vector3d::Zero &&
      this->selectedEntity.has_value())
  {
    math::Vector3d worldTorque =
      this->linkWorldPose.Rot().RotateVector(this->torque);
    math::Vector3d applicationPoint =
      inertialWorldPose.Pos();
    double scale = applicationPoint.Distance(this->camera->WorldPose().Pos())
      / 2.0;
    this->wrenchVis.UpdateVectorVisual(
      this->torqueVisual, worldTorque, applicationPoint, scale, false);
    this->torqueVisual->SetVisible(true);
  }
  else
  {
    this->torqueVisual->SetVisible(false);
  }

  // Update gizmo visual
  if (this->activeVector != RotationToolVector::NONE)
  {
    math::Vector3d pos;
    if (this->activeVector == RotationToolVector::FORCE
        && this->force != math::Vector3d::Zero)
    {
      pos =
        inertialWorldPose.Pos() +
        this->linkWorldPose.Rot().RotateVector(this->offset);
    }
    else if (this->activeVector == RotationToolVector::TORQUE
            && this->torque != math::Vector3d::Zero)
    {
      pos = inertialWorldPose.Pos();
    }
    else
    {
      this->activeVector = RotationToolVector::NONE;
      this->gizmoVisual->SetTransformMode(rendering::TransformMode::TM_NONE);
      this->gizmoVisual->SetActiveAxis(math::Vector3d::Zero);
      return;
    }

    double scale = pos.Distance(this->camera->WorldPose().Pos())
      / 2.0;

    // TODO(anyone): use GizmoVisual::LookAt function from
    // https://github.com/gazebosim/gz-rendering/pull/882

    // Update gizmo visual rotation so that they are always facing the
    // eye position and alligned with the active vector
    math::Quaterniond gizmoRot = this->linkWorldPose.Rot() * this->vectorRot;
    math::Vector3d eye = gizmoRot.RotateVectorReverse(
      (this->camera->WorldPosition() - pos).Normalized());

    rendering::VisualPtr xVis = this->gizmoVisual->ChildByAxis(
      rendering::TransformAxis::TA_ROTATION_X);
    xVis->SetVisible(false);

    rendering::VisualPtr yVis = this->gizmoVisual->ChildByAxis(
      rendering::TransformAxis::TA_ROTATION_Y);
    math::Vector3d yRot(0, atan2(eye.X(), eye.Z()), 0);
    math::Vector3d yRotOffset(GZ_PI * 0.5, -GZ_PI * 0.5, 0);
    yVis->SetWorldRotation(gizmoRot *
      math::Quaterniond(yRot) * math::Quaterniond(yRotOffset));

    rendering::VisualPtr zVis = this->gizmoVisual->ChildByAxis(
      rendering::TransformAxis::TA_ROTATION_Z);
    math::Vector3d zRot(0, 0, atan2(eye.Y(), eye.X()));
    zVis->SetWorldRotation(gizmoRot * math::Quaterniond(zRot));

    rendering::VisualPtr circleVis = this->gizmoVisual->ChildByAxis(
      rendering::TransformAxis::TA_ROTATION_Z << 1);
    math::Matrix4d lookAt;
    lookAt = lookAt.LookAt(this->camera->WorldPosition(), pos);
    math::Vector3d circleRotOffset(0, GZ_PI * 0.5, 0);
    circleVis->SetWorldRotation(
      lookAt.Rotation() * math::Quaterniond(circleRotOffset));

    this->gizmoVisual->SetLocalScale(1.5 * scale);
    this->gizmoVisual->SetTransformMode(rendering::TransformMode::TM_ROTATION);
    this->gizmoVisual->SetLocalPosition(pos);

    if (this->activeAxis == rendering::TransformAxis::TA_ROTATION_Y)
      this->gizmoVisual->SetActiveAxis(math::Vector3d::UnitY);
    else if (this->activeAxis == rendering::TransformAxis::TA_ROTATION_Z)
      this->gizmoVisual->SetActiveAxis(math::Vector3d::UnitZ);
    else
      this->gizmoVisual->SetActiveAxis(math::Vector3d::Zero);
  }
  else
  {
    this->gizmoVisual->SetTransformMode(rendering::TransformMode::TM_NONE);
    this->gizmoVisual->SetActiveAxis(math::Vector3d::Zero);
  }
}

/////////////////////////////////////////////////
void ApplyForceTorquePrivate::HandleMouseEvents()
{
  // check for mouse events
  if (!this->mouseDirty)
    return;
  this->mouseDirty = false;

  // handle mouse movements
  if (this->mouseEvent.Button() == common::MouseEvent::LEFT)
  {
    // Rotating vector around the clicked axis
    if (this->mouseEvent.Type() == common::MouseEvent::PRESS
        && this->activeVector != RotationToolVector::NONE)
    {
      this->mousePressPos = this->mouseEvent.Pos();

      // get the visual at mouse position
      rendering::VisualPtr visual = this->scene->VisualAt(
        this->camera,
        this->mouseEvent.Pos());

      if (visual)
      {
        // check if the visual is an axis in the gizmo visual
        auto axis = this->gizmoVisual->AxisById(visual->Id());
        if (axis == rendering::TransformAxis::TA_ROTATION_Y
            || axis == rendering::TransformAxis::TA_ROTATION_Z)
        {
          if (this->activeVector == RotationToolVector::FORCE)
            this->initialVector = this->force;
          else if (this->activeVector == RotationToolVector::TORQUE)
            this->initialVector = this->torque;
          else
            return;
          this->initialVectorRot = this->vectorRot;
          this->blockOrbit = true;
          this->sendBlockOrbit = true;
          this->activeAxis = axis;
        }
        else
        {
          this->blockOrbit = false;
          this->sendBlockOrbit = true;
          this->activeAxis = rendering::TransformAxis::TA_NONE;
          return;
        }
      }
    }
    else if (this->mouseEvent.Type() == common::MouseEvent::RELEASE)
    {
      this->blockOrbit = false;
      this->sendBlockOrbit = true;
      this->activeAxis = rendering::TransformAxis::TA_NONE;

      if (!this->mouseEvent.Dragging())
      {
        // get the visual at mouse position
        rendering::VisualPtr visual = this->scene->VisualAt(
          this->camera,
          this->mouseEvent.Pos());
        if (!visual)
          return;

        // Select a vector for the rotation tool
        auto id = visual->Id();
        if ((this->forceVisual->Id() == id ||
            this->forceVisual->ChildById(id))
            && this->activeVector != RotationToolVector::FORCE)
        {
          this->activeVector = RotationToolVector::FORCE;
          this->vectorRot = math::Matrix4d::LookAt(
            -this->force, math::Vector3d::Zero).Rotation();
        }
        else if ((this->torqueVisual->Id() == id ||
                this->torqueVisual->ChildById(id))
                && this->activeVector != RotationToolVector::TORQUE)
        {
          this->activeVector = RotationToolVector::TORQUE;
          this->vectorRot = math::Matrix4d::LookAt(
            -this->torque, math::Vector3d::Zero).Rotation();
        }
        else if (this->gizmoVisual->AxisById(visual->Id()) ==
                rendering::TransformAxis::TA_NONE)
        {
          this->activeVector = RotationToolVector::NONE;
        }
      }
    }
  }
  if (this->mouseEvent.Type() == common::MouseEvent::MOVE
      && this->activeAxis != rendering::TransformAxis::TA_NONE)
  {
    this->blockOrbit = true;
    this->sendBlockOrbit = true;

    auto imageWidth = static_cast<double>(this->camera->ImageWidth());
    auto imageHeight = static_cast<double>(this->camera->ImageHeight());
    double nx = 2.0 * this->mousePressPos.X() / imageWidth - 1.0;
    double ny = 1.0 - 2.0 * this->mousePressPos.Y() / imageHeight;
    double nxEnd = 2.0 * this->mouseEvent.Pos().X() / imageWidth - 1.0;
    double nyEnd = 1.0 - 2.0 * this->mouseEvent.Pos().Y() / imageHeight;
    math::Vector2d start(nx, ny);
    math::Vector2d end(nxEnd, nyEnd);

    // Axis of rotation in world frame
    math::Vector3d axis;
    if (this->activeAxis == rendering::TransformAxis::TA_ROTATION_Y)
    {
      axis = this->linkWorldPose.Rot().RotateVector(this->vectorRot.YAxis());
    }
    else if (this->activeAxis == rendering::TransformAxis::TA_ROTATION_Z)
    {
      axis = this->linkWorldPose.Rot().RotateVector(this->vectorRot.ZAxis());
    }

    /// get start and end pos in world frame from 2d point
    math::Pose3d inertialWorldPose = this->linkWorldPose * this->inertialPose;
    math::Vector3d pos = inertialWorldPose.Pos() +
      this->linkWorldPose.Rot().RotateVector(this->offset);
    double d = pos.Dot(axis);
    math::Planed plane(axis, d);

    math::Vector3d startPos;
    this->ray->SetFromCamera(this->camera, start);
    if (auto v{plane.Intersection(
      this->ray->Origin(), this->ray->Direction())})
      startPos = *v;
    else
      return;

    math::Vector3d endPos;
    this->ray->SetFromCamera(this->camera, end);
    if (auto v{plane.Intersection(
      this->ray->Origin(), this->ray->Direction())})
      endPos = *v;
    else
      return;

    // get vectors from link pos to both points
    startPos = (startPos - pos).Normalized();
    endPos = (endPos - pos).Normalized();
    // compute angle between two vectors
    double signTest = startPos.Cross(endPos).Dot(axis);
    double angle = atan2(
      (startPos.Cross(endPos)).Length(), startPos.Dot(endPos));
    if (signTest < 0)
      angle = -angle;

    // Desired rotation in link frame
    axis = this->linkWorldPose.Rot().RotateVectorReverse(axis);
    math::Quaterniond rot(axis, angle);
    this->vectorRot = rot * this->initialVectorRot;
    math::Vector3d newVector =
      this->initialVector.Length() * this->vectorRot.XAxis();

    if (this->activeVector == RotationToolVector::FORCE)
      this->force = newVector;
    else if (this->activeVector == RotationToolVector::TORQUE)
      this->torque = newVector;
    this->vectorDirty = true;
  }
}

// Register this plugin
GZ_ADD_PLUGIN(ApplyForceTorque, gz::gui::Plugin)
