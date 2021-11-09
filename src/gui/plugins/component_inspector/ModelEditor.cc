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
#include <vector>
#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>

#include <sdf/Link.hh>
#include <sdf/Sensor.hh>
#include <sdf/parser.hh>

#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
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

    /// \brief Entity URI, such as a URI for a mesh.
    public: std::string uri;
  };

  class ModelEditorPrivate
  {
    /// \brief Handle entity addition
    /// \param[in] _geomOrLightType Geometry or light type, e.g. sphere,
    /// directional, etc
    /// \param[in] _entityType Type of entity: link, visual, collision, etc
    /// \param[in] _parentEntity Parent entity
    /// \param[in] _uri URI associated with the entity, needed for mesh
    /// types.
    public: void HandleAddEntity(const std::string &_geomOrLightType,
        const std::string &_entityType, Entity _parentEntity,
        const std::string &_uri);

    /// \brief Get a SDF string of a geometry
    /// \param[in] _eta Entity to add.
    public: std::string GeomSDFString(const EntityToAdd &_eta) const;

    /// \brief Get a SDF string of a light
    /// \param[in] _eta Entity to add.
    public: std::string LightSDFString(const EntityToAdd &_eta) const;

    /// \brief Get a SDF string of a link
    /// \param[in] _eta Entity to add.
    /// \return SDF string
    public: std::string LinkSDFString(const EntityToAdd &_eta) const;

    /// \brief Create a link
    /// \param[in] _eta Entity to add.
    public: std::optional<sdf::Link> CreateLink(
                const EntityToAdd &_eta,
                EntityComponentManager &_ecm) const;

    /// \brief Create a sensor
    /// \param[in] _eta Entity to add.
    public: std::optional<sdf::Sensor> CreateSensor(
                const EntityToAdd &_eta,
                EntityComponentManager &_ecm) const;

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


    /// \brief Sensors to add to the ECM
    public: std::vector<sdf::Sensor> sensorsToAdd;

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
  std::set<Entity> newEntities;
  for (const auto &eta: this->dataPtr->entitiesToAdd)
  {
    Entity entity = kNullEntity;

    if (eta.entityType == "link")
    {
      std::optional<sdf::Link> link = this->dataPtr->CreateLink(eta, _ecm);
      if (link)
      {
        entity = this->dataPtr->entityCreator->CreateEntities(&(*link));
        this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
      }
    }
    else if (eta.entityType == "sensor")
    {
      std::optional<sdf::Sensor> sensor =
        this->dataPtr->CreateSensor(eta, _ecm);
      if (sensor)
      {
        entity = this->dataPtr->entityCreator->CreateEntities(&(*sensor));
        this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
      }
    }

    // If an entity was created, then traverse the tree and add all new
    // entities created by the entity creator to the set
    if (entity != kNullEntity)
    {
      std::list<Entity> entities;
      entities.push_back(entity);
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
    }
  }

  // use tmp AddedRemovedEntities event to update other gui plugins
  // note this event will be removed in Ignition Garden
  std::set<Entity> removedEntities;
  ignition::gazebo::gui::events::AddedRemovedEntities event(
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
      this->dataPtr->HandleAddEntity(event->Entity().toStdString(),
          event->EntityType().toStdString(),
          event->ParentEntity(),
          event->Uri().toStdString());
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
      << "  <uri>" << _eta.uri << "</uri>"
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
std::optional<sdf::Link> ModelEditorPrivate::CreateLink(
    const EntityToAdd &_eta, EntityComponentManager &_ecm) const
{
  sdf::Link linkSdf;
  if (_eta.parentEntity == kNullEntity)
  {
    ignerr << "Parent entity not defined." << std::endl;
    return std::nullopt;
  }

  // create an sdf::Link to it can be added to the ECM throught the
  // CreateEntities call
  std::string linkSDFStr = this->LinkSDFString(_eta);
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
    return std::nullopt;
  }

  // generate unique link name
  // note passing components::Link() as arg to EntityByComponents causes
  // a crash on exit, see issue #1158
  std::string linkName = "link";
  Entity linkEnt = _ecm.EntityByComponents(
      components::ParentEntity(_eta.parentEntity),
      components::Name(linkName));
  int64_t counter = 0;
  while (linkEnt)
  {
    linkName = std::string("link") + "_" + std::to_string(++counter);
    linkEnt = _ecm.EntityByComponents(
        components::ParentEntity(_eta.parentEntity),
        components::Name(linkName));
  }

  linkSdf.SetName(linkName);
  return linkSdf;
}

/////////////////////////////////////////////////
std::optional<sdf::Sensor> ModelEditorPrivate::CreateSensor(
    const EntityToAdd &_eta, EntityComponentManager &_ecm) const
{
  // Exit early if there is no parent entity
  if (_eta.parentEntity == kNullEntity)
  {
    ignerr << "Parent entity not defined." << std::endl;
    return std::nullopt;
  }

  sdf::Sensor sensor;

  std::string type;

  // Replace spaces with underscores.
  common::replaceAll(type, _eta.geomOrLightType, " ", "_");

  std::ostringstream stream;
  stream << "<sdf version='" << SDF_VERSION << "'>"
    << "<sensor name='" << type << "' type='" << type << "'>"
    << "<" << type << "></" << type << "></sensor></sdf>";

  auto sdfStr = stream.str();
  sdf::ElementPtr sensorElem(new sdf::Element);
  sdf::initFile("sensor.sdf", sensorElem);
  sdf::readString(sdfStr, sensorElem);
  sensor.Load(sensorElem);

  // generate unique sensor name
  // note passing components::Link() as arg to EntityByComponents causes
  // a crash on exit, see issue #1158
  std::string sensorName = type;
  Entity sensorEnt = _ecm.EntityByComponents(
      components::ParentEntity(_eta.parentEntity),
      components::Name(sensorName));
  int64_t counter = 0;
  while (sensorEnt)
  {
    sensorName = type + "_" + std::to_string(++counter);
    sensorEnt = _ecm.EntityByComponents(
        components::ParentEntity(_eta.parentEntity),
        components::Name(sensorName));
  }
  sensor.SetName(sensorName);

  sensor.SetTopic("/" + sensorName);

  return sensor;
}

/////////////////////////////////////////////////
void ModelEditorPrivate::HandleAddEntity(const std::string &_geomOrLightType,
  const std::string &_type, Entity _parentEntity,
  const std::string &_uri)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::string entType = common::lowercase(_type);
  std::string geomLightType = common::lowercase(_geomOrLightType);

  EntityToAdd eta;
  eta.entityType = entType;
  eta.geomOrLightType = geomLightType;
  eta.parentEntity = _parentEntity;
  eta.uri = _uri;
  this->entitiesToAdd.push_back(eta);
}
