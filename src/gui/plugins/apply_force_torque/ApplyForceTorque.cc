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

#include <mutex>
#include <string>

#include <gz/gui/Application.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SystemPluginInfo.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/transport/Node.hh>

#include "ApplyForceTorque.hh"

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate
  {
    /// \brief Publish EntityWrench messages in order to apply force and torque
    /// \param[in] _applyForce True if the force should be applied
    /// \param[in] _applyTorque True if the torque should be applied
    public: void PublishWrench(bool _applyForce, bool _applyTorque);

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

    /// \brief Force to be applied in link-fixed frame''
    public: math::Vector3d force{0.0, 0.0, 0.0};

    /// \brief Torque to be applied in link-fixed frame
    public: math::Vector3d torque{0.0, 0.0, 0.0};

    /// \brief Offset from the link origin to the center of mass in world coords
    public: math::Vector3d inertialPos;

    /// \brief Orientation of the link-fixed frame
    public: math::Quaterniond linkRot;
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
ApplyForceTorque::~ApplyForceTorque() = default;

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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (_event->type() ==
    gz::sim::gui::events::EntitiesSelected::kType)
  {
    gz::sim::gui::events::EntitiesSelected *_e =
        static_cast<gz::sim::gui::events::EntitiesSelected*>(_event);
    this->dataPtr->selectedEntity = _e->Data().front();
    this->dataPtr->changedEntity = true;
  }
  else if (_event->type() ==
    gz::sim::gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntity.reset();
    this->dataPtr->changedEntity = true;
  }

  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ApplyForceTorque::Update(const UpdateInfo &/*_info*/,
  EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Load the ApplyLinkWrench system
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
      this->dataPtr->inertialPos =
        linkWorldPose.Rot().RotateVector(inertial->Data().Pose().Pos());
      this->dataPtr->linkRot = linkWorldPose.Rot();
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
void ApplyForceTorque::UpdateForce(double _x, double _y, double _z)
{
  this->dataPtr->force = {_x, _y, _z};
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateTorque(double _x, double _y, double _z)
{
  this->dataPtr->torque = {_x, _y, _z};
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
  math::Vector3d forceToApply = this->linkRot.RotateVector(
    _applyForce ? this->force : math::Vector3d::Zero);
  math::Vector3d torqueToApply = this->linkRot.RotateVector(
    _applyTorque ? this->torque : math::Vector3d::Zero) +
    this->inertialPos.Cross(forceToApply);

  msgs::EntityWrench msg;
  msg.mutable_entity()->set_id(*this->selectedEntity);
  msgs::Set(msg.mutable_wrench()->mutable_force(), forceToApply);
  msgs::Set(msg.mutable_wrench()->mutable_torque(), torqueToApply);

  this->pub.Publish(msg);
}

// Register this plugin
GZ_ADD_PLUGIN(ApplyForceTorque, gz::gui::Plugin)
