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

#include <iostream>
#include <mutex>
#include <vector>

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/sim/gui/GuiEvents.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/msgs/Utility.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "ApplyForceTorque.hh"

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate
  {
    /// \brief Publish wrench messages
    public: void Publish(bool _applyForce, bool _applyTorque);

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher for EntityWrench messages
    public: transport::Node::Publisher pub;

    /// \brief A mutex to protect wrenches
    public: std::mutex mutex;

    /// \brief Name of the selected model
    public: QString modelName;

    /// \brief List of the name of the links in the selected model
    public: QStringList linkNameList;

    /// \brief List of links in the selected model
    public: std::vector<Link> linkList;

    /// \brief Index of selected link in list
    public: int linkIndex{-1};

    /// \brief Currently selected entities, organized by order of selection.
    public: std::vector<Entity> selectedEntities;

    /// \brief True if a new entity was selected
    public: bool changedEntity{false};

    /// \brief Force to be applied
    public: math::Vector3d force{0.0, 0.0, 0.0};

    /// \brief Offset of the force application relative to center of mass
    public: math::Vector3d offset{0.0, 0.0, 0.0};

    /// \brief Torque to be applied
    public: math::Vector3d torque{0.0, 0.0, 0.0};
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

  gz::gui::App()->findChild<gz::gui::MainWindow *>
      ()->installEventFilter(this);
  gz::gui::App()->findChild<gz::gui::MainWindow *>
      ()->QuickWindow()->installEventFilter(this);
}

/////////////////////////////////////////////////
bool ApplyForceTorque::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() ==
    gz::sim::gui::events::EntitiesSelected::kType)
  {
    gz::sim::gui::events::EntitiesSelected *_e =
        static_cast<gz::sim::gui::events::EntitiesSelected*>(_event);
    this->dataPtr->selectedEntities = _e->Data();
    this->dataPtr->changedEntity = true;
  }
  else if (_event->type() ==
    gz::sim::gui::events::DeselectAllEntities::kType)
  {
    this->dataPtr->selectedEntities.clear();
    this->dataPtr->changedEntity = true;
  }

  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ApplyForceTorque::PreUpdate(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // if (this->dataPtr->applyForce)
  // {
  //   std::string entityName = "box";
  //   auto entities = entitiesFromScopedName(entityName, _ecm);
  //   if (entities.empty())
  //   {
  //     gzerr << "No entity named [" << entityName << "]" << std::endl;
  //   }
  //   auto entity = *entities.begin();

  //   Model model(entity);
  //   if (!model.Valid(_ecm))
  //   {
  //     gzerr << "Entity is not a model." << std::endl;
  //   }

  //   Link link(model.CanonicalLink(_ecm));
  //   math::Vector3d force{10000.0, 0.0, 0.0};
  //   link.AddWorldForce(_ecm, force);

  //   this->dataPtr->applyForce = false;
  //   gzdbg << "Applied force to " << entityName << std::endl;
  // }
}

/////////////////////////////////////////////////
void ApplyForceTorque::Update(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    if (!this->dataPtr->changedEntity)
    {
      return;
    }

    this->dataPtr->changedEntity = false;
    if (this->dataPtr->selectedEntities.empty())
    {
      this->dataPtr->modelName = "";
      this->dataPtr->linkList.clear();
      this->dataPtr->linkIndex = -1;
    }
    else
    {
      auto entity = this->dataPtr->selectedEntities.front();
      Model modelSelected(entity);
      Link linkSelected(entity);
      // What happens if the entity is neither a Link nor a Model?
      if (modelSelected.Valid(_ecm))
      {
        linkSelected = Link(modelSelected.CanonicalLink(_ecm));
      }
      else if (linkSelected.Valid(_ecm))
      {
        modelSelected = *linkSelected.ParentModel(_ecm);
      }

      this->dataPtr->modelName = QString::fromStdString(
        modelSelected.Name(_ecm));

      this->dataPtr->linkList.clear();
      this->dataPtr->linkNameList.clear();

      auto links = modelSelected.Links(_ecm);
      unsigned int i{0};
      while (i < modelSelected.LinkCount(_ecm))
      {
        Link link(links[i]);
        this->dataPtr->linkList.push_back(link);
        this->dataPtr->linkNameList.push_back(
          QString::fromStdString(*link.Name(_ecm)));

        if (link.Entity() == linkSelected.Entity())
        {
          this->dataPtr->linkIndex = i;
        }
        ++i;
      }

      gzdbg << "modelName: " << this->dataPtr->modelName.toStdString() <<
        ", link: " << *linkSelected.Name(_ecm) <<
        ", linkIndex: " << this->dataPtr->linkIndex << std::endl;

      // Create publisher if not yet created
      if (!this->dataPtr->pub.Valid())
      {
        std::string worldName{""};
        _ecm.Each<components::World, components::Name>(
            [&](const Entity &_entity,
              const components::World * /* _world */ ,
              const components::Name *_name)->bool
            {
              World world(_entity);
              for (auto &model : world.Models(_ecm))
              {
                if (model == entity)
                {
                  worldName = _name->Data();
                  return true;
                }
              }
              gzerr << "World not found" << std::endl;
              return false;
            });
        auto topic = transport::TopicUtils::AsValidTopic(
          "/world/" + worldName + "/wrench");
        this->dataPtr->pub =
            this->dataPtr->node.Advertise<msgs::EntityWrench>(topic);
        gzdbg << "Created publisher to " << topic << std::endl;
      }
    }
  }

  this->ModelNameChanged();
  this->LinkNameListChanged();
  this->LinkIndexChanged();
}

/////////////////////////////////////////////////
QString ApplyForceTorque::ModelName() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->modelName;
}

/////////////////////////////////////////////////
QStringList ApplyForceTorque::LinkNameList() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->linkNameList;
}

/////////////////////////////////////////////////
int ApplyForceTorque::LinkIndex() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->linkIndex;
}

/////////////////////////////////////////////////
void ApplyForceTorque::SetLinkIndex(int _linkIndex)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->linkIndex = _linkIndex;
  gzdbg << "linkIndex: " << this->dataPtr->linkIndex << std::endl;
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateForce(double _x, double _y, double _z)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->force = {_x, _y, _z};
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateOffset(double _x, double _y, double _z)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->offset = {_x, _y, _z};
}

/////////////////////////////////////////////////
void ApplyForceTorque::UpdateTorque(double _x, double _y, double _z)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->torque = {_x, _y, _z};
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyForce()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gzdbg << "Force: (" << this->dataPtr->force[0] << ", " <<
          this->dataPtr->force[1] << ", " <<
          this->dataPtr->force[2] <<
          ")" << std::endl;
  this->dataPtr->Publish(true, false);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyTorque()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gzdbg << "Torque: (" << this->dataPtr->torque[0] << ", " <<
          this->dataPtr->torque[1] << ", " <<
          this->dataPtr->torque[2] <<
          ")" << std::endl;
  this->dataPtr->Publish(false, true);
}

/////////////////////////////////////////////////
void ApplyForceTorque::ApplyAll()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  gzdbg << "Force: (" << this->dataPtr->force[0] << ", " <<
          this->dataPtr->force[1] << ", " <<
          this->dataPtr->force[2] <<
          ") Torque: (" << this->dataPtr->torque[0] << ", " <<
          this->dataPtr->torque[1] << ", " <<
          this->dataPtr->torque[2] <<
          ")" << std::endl;
  this->dataPtr->Publish(true, true);
}

/////////////////////////////////////////////////
void ApplyForceTorquePrivate::Publish(bool _applyForce, bool _applyTorque)
{
  if (this->linkIndex == -1)
  {
    gzdbg << "No link selected" << std::endl;
    return;
  }

  Entity entity = this->linkList.at(this->linkIndex).Entity();
  if (entity == kNullEntity)
  {
    gzdbg << "Invalid link" << std::endl;
    return;
  }

  msgs::EntityWrench msg;
  msg.mutable_entity()->set_id(entity);
  msgs::Set(msg.mutable_wrench()->mutable_force(),
      _applyForce ? this->force : math::Vector3d::Zero);
  msgs::Set(msg.mutable_wrench()->mutable_force_offset(),
      _applyForce ? this->offset : math::Vector3d::Zero);
  msgs::Set(msg.mutable_wrench()->mutable_torque(),
      _applyTorque ? this->torque : math::Vector3d::Zero);

  this->pub.Publish(msg);
}

// Register this plugin
GZ_ADD_PLUGIN(ApplyForceTorque,
              gz::gui::Plugin,
              System,
              ApplyForceTorque::ISystemPreUpdate);
