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

#include <iostream>
#include <list>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

#include <sdf/Link.hh>
#include <sdf/parser.hh>

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Recreate.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ModelEditor.hh"

namespace ignition::gazebo
{
  class EntityToAdd
  {
    /// \brief Entity to add to the model editor
    public: std::string geomOrLightType;

    /// \brief Type of entity to add
    public: std::string entityType;

    /// \brief Parent entity to add the entity to
    public: Entity parentEntity;

    /// \brief Additional entity-specific data needed
    public: std::unordered_map<std::string, std::string> data;
  };

  class ModelEditorPrivate
  {
    /// \brief Handle entity addition
    /// \param[in] _geomOrLightType Geometry or light type, e.g. sphere,
    /// directional, etc
    /// \param[in] _entityType Type of entity: link, visual, collision, etc
    /// \param[in] _parentEntity Name of parent entity
    /// \param[in] _data Additional variable data for specific instances
    public: void HandleAddEntity(const std::string &_geomOrLightType,
        const std::string &_entityType, Entity _parentEntity,
        const std::unordered_map<std::string, std::string> &_data);

    /// \brief Get a SDF string of a geometry
    /// \param[in] _eta Entity to add.
    public: std::string GeomSDFString(const EntityToAdd &_eta) const;

    /// \brief Get a SDF string of a light
    /// \param[in] _eta Entity to add.
    public: std::string LightSDFString(const EntityToAdd &_eta) const;

    /// \brief Get a SDF string of a link
    /// \param[in] _eta Entity to add.
    public: std::string LinkSDFString(const EntityToAdd &_eta) const;

    /// \brief Get a SDF string of a link
    /// \param[in] _eta Entity to add.
    public: sdf::ElementPtr JointSDF(const EntityToAdd &_eta) const;

    /// \brief Entity Creator API.
    public: std::unique_ptr<SdfEntityCreator> entityCreator{nullptr};

    /// \brief A record of the ids in the editor
    /// for easy deletion of visuals later
    public: std::vector<Entity> entityIds;

    /// \brief Mutex to protect the entity sdf list
    public: std::mutex mutex;

    /// \brief A map of links to add to the ECM and the parent entity names
    // public: std::vector<std::pair<sdf::Link, std::string>> linksToAdd;
    public: std::vector<EntityToAdd> entitiesToAdd;

    /// \brief Event Manager
    public: EventManager eventMgr;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
ModelEditor::ModelEditor()
  : dataPtr(std::make_unique<ModelEditorPrivate>())
{
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor() = default;

/////////////////////////////////////////////////
void ModelEditor::Load()
{
  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void ModelEditor::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ModelEditor::Update");

  if (!this->dataPtr->entityCreator)
  {
    this->dataPtr->entityCreator = std::make_unique<SdfEntityCreator>(
        _ecm, this->dataPtr->eventMgr);
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // add link entities to the ECM
  std::list<Entity> entities;
  std::set<Entity> newEntities;

  for (const auto &eta : this->dataPtr->entitiesToAdd)
  {
    if (eta.parentEntity == kNullEntity)
    {
      ignerr << "Parent entity not defined." << std::endl;
       continue;
    }

    if (eta.entityType == "link")
    {
      sdf::Link linkSdf;
      // create an sdf::Link to it can be added to the ECM throught the
      // CreateEntities call
      std::string linkSDFStr = this->dataPtr->LinkSDFString(eta);
      if (!linkSDFStr.empty())
      {
        linkSDFStr = std::string("<sdf version='") + SDF_VERSION + "'>" +
            linkSDFStr + "</sdf>";

        sdf::ElementPtr linkElem(new sdf::Element);
        sdf::initFile("link.sdf", linkElem);
        sdf::readString(linkSDFStr, linkElem);
        linkSdf.Load(linkElem);
      }
      else
      {
        continue;
      }

      // generate unique link name
      // note passing components::Link() as arg to EntityByComponents causes
      // a crash on exit, see issue #1158
      std::string linkName = "link";
      Entity linkEnt = _ecm.EntityByComponents(
          components::ParentEntity(eta.parentEntity),
          components::Name(linkName));
      int64_t counter = 0;
      while (linkEnt)
      {
        linkName = std::string("link") + "_" + std::to_string(++counter);
        linkEnt = _ecm.EntityByComponents(
            components::ParentEntity(eta.parentEntity),
            components::Name(linkName));
      }

      linkSdf.SetName(linkName);
      auto entity = this->dataPtr->entityCreator->CreateEntities(&linkSdf);
      this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
      // Make sure to mark the parent as needing recreation. This will
      // tell the server to rebuild the model with the new link.
      _ecm.CreateComponent(eta.parentEntity, components::Recreate());

      // traverse the tree and add all new entities created by the entity
      // creator to the set
      entities.push_back(entity);
    }
    else if (eta.entityType == "joint")
    {
      sdf::ElementPtr jointElem = this->dataPtr->JointSDF(eta);
      if (nullptr == jointElem)
        continue;

      std::string jointName = "joint";
      Entity jointEnt = _ecm.EntityByComponents(
          components::ParentEntity(eta.parentEntity),
          components::Name(jointName));
      int64_t counter = 0;
      while (jointEnt)
      {
        jointName = std::string("joint") + "_" + std::to_string(++counter);
        jointEnt = _ecm.EntityByComponents(
            components::ParentEntity(eta.parentEntity),
            components::Name(jointName));
      }

      jointElem->SetName(jointName);

      sdf::Joint jointSdf;
      jointSdf.Load(jointElem);
      auto entity =
        this->dataPtr->entityCreator->CreateEntities(&jointSdf, true);
      this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
      // Make sure to mark the parent as needing recreation. This will
      // tell the server to rebuild the model with the new link.
      _ecm.CreateComponent(eta.parentEntity, components::Recreate());

      // traverse the tree and add all new entities created by the entity
      // creator to the set
      entities.push_back(entity);
    }
  }

  // traverse the tree and add all new entities created by the entity
  // creator to the set
  while (!entities.empty())
  {
    Entity ent = entities.front();
    entities.pop_front();

    // add new entity created
    newEntities.insert(ent);

    auto childEntities = _ecm.EntitiesByComponents(
        components::ParentEntity(ent));
    for (const auto &child : childEntities)
      entities.push_back(child);
  }

  // use tmp GuiNewRemovedEntities  event to update other gui plugins
  // note this event will be removed in Ignition Garden
  std::set<Entity> removedEntities;
  ignition::gazebo::gui::events::GuiNewRemovedEntities event(
      newEntities, removedEntities);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &event);

  this->dataPtr->entitiesToAdd.clear();
}

/////////////////////////////////////////////////
bool ModelEditor::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gazebo::gui::events::ModelEditorAddEntity::kType)
  {
    auto event = reinterpret_cast<gui::events::ModelEditorAddEntity *>(_event);
    if (event)
    {
      // Convert to an unordered map of STL strings for convenience
      std::unordered_map<std::string, std::string> data;
      for (auto key : event->data.toStdMap())
      {
        data[key.first.toStdString()] = key.second.toStdString();
      }

      this->dataPtr->HandleAddEntity(event->Entity().toStdString(),
          event->EntityType().toStdString(),
          event->ParentEntity(), data);
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::LightSDFString(const EntityToAdd &_eta) const
{
  std::stringstream lightStr;
  lightStr << "<light name='light' type='" << _eta.geomOrLightType << "'>";

  if (_eta.geomOrLightType == "directional")
  {
    lightStr
        << "<cast_shadows>false</cast_shadows>"
        << "<diffuse>1.0 1.0 1.0 1</diffuse>"
        << "<specular>0.5 0.5 0.5 1</specular>";
  }
  else if (_eta.geomOrLightType == "spot")
  {
    lightStr
        << "<cast_shadows>false</cast_shadows>"
        << "<diffuse>1.0 1.0 1.0 1</diffuse>"
        << "<specular>0.5 0.5 0.5 1</specular>"
        << "<attenuation>"
        <<   "<range>4</range>"
        <<   "<constant>0.2</constant>"
        <<   "<linear>0.5</linear>"
        <<   "<quadratic>0.01</quadratic>"
        << "</attenuation>"
        << "<direction>0 0 -1</direction>"
        << "<spot>"
        <<   "<inner_angle>0.1</inner_angle>"
        <<   "<outer_angle>0.5</outer_angle>"
        <<   "<falloff>0.8</falloff>"
        << "</spot>";
  }
  else if (_eta.geomOrLightType == "point")
  {
    lightStr
        << "<cast_shadows>false</cast_shadows>"
        << "<diffuse>1.0 1.0 1.0 1</diffuse>"
        << "<specular>0.5 0.5 0.5 1</specular>"
        << "<attenuation>"
        <<   "<range>4</range>"
        <<   "<constant>0.2</constant>"
        <<   "<linear>0.5</linear>"
        <<   "<quadratic>0.01</quadratic>"
        << "</attenuation>";
  }
  else
  {
    ignwarn << "Light type not supported: "
      << _eta.geomOrLightType << std::endl;
    return std::string();
  }

  lightStr << "</light>";
  return lightStr.str();
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::GeomSDFString(const EntityToAdd &_eta) const
{
  math::Vector3d size = math::Vector3d::One;
  std::stringstream geomStr;
  geomStr << "<geometry>";
  if (_eta.geomOrLightType == "box")
  {
    geomStr
      << "<box>"
      << "  <size>" << size << "</size>"
      << "</box>";
  }
  else if (_eta.geomOrLightType == "sphere")
  {
    geomStr
      << "<sphere>"
      << "  <radius>" << size.X() * 0.5 << "</radius>"
      << "</sphere>";
  }
  else if (_eta.geomOrLightType == "cylinder")
  {
    geomStr
      << "<cylinder>"
      << "  <radius>" << size.X() * 0.5 << "</radius>"
      << "  <length>" << size.Z() << "</length>"
      << "</cylinder>";
  }
  else if (_eta.geomOrLightType == "capsule")
  {
    geomStr
      << "<capsule>"
      << "  <radius>" << size.X() * 0.5 << "</radius>"
      << "  <length>" << size.Z() << "</length>"
      << "</capsule>";
  }
  else if (_eta.geomOrLightType == "ellipsoid")
  {
    geomStr
      << "<ellipsoid>"
      << "  <radii>" << size * 0.5 << "</radii>"
      << "</ellipsoid>";
  }
  else if (_eta.geomOrLightType == "mesh")
  {
    geomStr
      << "<mesh>"
      << "  <uri>" << _eta.data.at("uri") << "</uri>"
      << "</mesh>";
  }
  else
  {
    ignwarn << "Geometry type not supported: "
      << _eta.geomOrLightType << std::endl;
    return std::string();
  }


  geomStr << "</geometry>";
  return geomStr.str();
}

/////////////////////////////////////////////////
sdf::ElementPtr ModelEditorPrivate::JointSDF(const EntityToAdd &_eta) const
{
  std::unordered_set<std::string> validJointTypes = {
    "revolute", "ball", "continuous", "fixed", "gearbox", "prismatic",
    "revolute2", "screw", "universal"};

  if (validJointTypes.count(_eta.geomOrLightType) == 0)
  {
    ignwarn << "Joint type not supported: "
      << _eta.geomOrLightType << std::endl;
    return nullptr;
  }

  auto joint = std::make_shared<sdf::Element>();
  sdf::initFile("joint.sdf", joint);

  joint->GetAttribute("name")->Set("joint");
  joint->GetAttribute("type")->Set(_eta.geomOrLightType);
  joint->GetElement("parent")->Set(_eta.data.at("parent_link"));
  joint->GetElement("child")->Set(_eta.data.at("child_link"));

  return joint;
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::LinkSDFString(const EntityToAdd &_eta) const
{
  std::stringstream linkStr;
  if (_eta.geomOrLightType == "empty")
  {
    linkStr << "<link name='link'/>";
    return linkStr.str();
  }

  std::string geomOrLightStr;
  if (_eta.geomOrLightType == "spot" || _eta.geomOrLightType == "directional" ||
      _eta.geomOrLightType == "point")
  {
    geomOrLightStr = this->LightSDFString(_eta);
    linkStr
        << "<link name='link'>"
        << geomOrLightStr
        << "</link>";
  }
  else
  {
    geomOrLightStr = this->GeomSDFString(_eta);
    linkStr
        << "<link name='link'>"
        << "  <visual name='visual'>"
        << geomOrLightStr
        << "  </visual>"
        << "  <collision name='collision'>"
        << geomOrLightStr
        << "  </collision>"
        << "</link>";
  }

  if (geomOrLightStr.empty())
    return std::string();

  return linkStr.str();
}

/////////////////////////////////////////////////
void ModelEditorPrivate::HandleAddEntity(const std::string &_geomOrLightType,
  const std::string &_type, Entity _parentEntity,
  const std::unordered_map<std::string, std::string> &_data)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::string entType = common::lowercase(_type);
  std::string geomLightType = common::lowercase(_geomOrLightType);

  EntityToAdd eta;
  eta.entityType = entType;
  eta.geomOrLightType = geomLightType;
  eta.parentEntity = _parentEntity;
  eta.data = _data;

  this->entitiesToAdd.push_back(eta);
}
