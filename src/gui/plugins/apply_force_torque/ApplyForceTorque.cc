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

#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Helpers.hh>
#include <gz/sim/gui/GuiEvents.hh>
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
#include <gz/math/Vector3.hh>

#include "ApplyForceTorque.hh"

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate
  {
    /// \brief Publish wrench messages
    public: void PublishWrench(bool _applyForce, bool _applyTorque);

    /// \brief Transport node
    public: transport::Node node;

    /// \brief Publisher for EntityWrench messages
    public: transport::Node::Publisher pub;

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

    /// \brief Force to be applied
    public: math::Vector3d force{0.0, 0.0, 0.0};

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
bool ApplyForceTorque::eventFilter(QObject *_obj, QEvent *_event)
{
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
  if (!this->selectedEntity.has_value())
  {
    gzdbg << "No link selected" << std::endl;
    return;
  }

  math::Vector3d forceToApply =
    _applyForce ? this->force : math::Vector3d::Zero;
  math::Vector3d torqueToApply =
    _applyTorque ? this->torque : math::Vector3d::Zero;

  gzdbg << "Applying wrench [" <<
    forceToApply[0] << " " <<
    forceToApply[1] << " " <<
    forceToApply[2] << " " <<
    torqueToApply[0] << " " <<
    torqueToApply[1] << " " <<
    torqueToApply[2] << "] to entity [" <<
    *this->selectedEntity << "]" << std::endl;

  msgs::EntityWrench msg;
  msg.mutable_entity()->set_id(*this->selectedEntity);
  msgs::Set(msg.mutable_wrench()->mutable_force(), forceToApply);
  msgs::Set(msg.mutable_wrench()->mutable_torque(), torqueToApply);

  this->pub.Publish(msg);
}

// Register this plugin
GZ_ADD_PLUGIN(ApplyForceTorque, gz::gui::Plugin);
