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
#include <ignition/plugin/Register.hh>

#include <sdf/Link.hh>
#include <sdf/Sensor.hh>
#include <sdf/AirPressure.hh>
#include <sdf/Altimeter.hh>
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

    /// \brief Name of parent entity to add the entity to
    public: Entity parentEntity;
  };

  class ModelEditorPrivate
  {
    /// \brief Handle entity addition
    /// \param[in] _geomOrLightType Geometry or light type, e.g. sphere, directional, etc
    /// \param[in] _entityType Type of entity: link, visual, collision, etc
    /// \param[in] _parentEntity Parent entity
    public: void HandleAddEntity(const std::string &_geomOrLightType,
        const std::string &_entityType, Entity _parentEntity);

    /// \brief Get a SDF string of a geometry
    /// \param[in] _geomType Type of geometry
    public: std::string GeomSDFString(
        const std::string &_geomType) const;

    /// \brief Get a SDF string of a light
    /// \param[in] _lightType Type of light
    public: std::string LightSDFString(
        const std::string &_lightType) const;

    /// \brief Get a SDF string of a link
    /// \param[in] _geomOrLightType Type of light or geometry
    public: std::string LinkSDFString(
        const std::string &_geomOrLightType) const;

    public: sdf::Sensor CreateSensor(const std::string &_type) const;

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
  : GuiSystem(), dataPtr(std::make_unique<ModelEditorPrivate>())
{
}

/////////////////////////////////////////////////
ModelEditor::~ModelEditor() = default;

/////////////////////////////////////////////////
void ModelEditor::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Model editor";

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
  for (auto & eta: this->dataPtr->entitiesToAdd)
  {
    sdf::Link linkSdf;
    if (eta.entityType == "link")
    {
      // create an sdf::Link to it can be added to the ECM throught the
      // CreateEntities call
      std::string linkSDFStr = this->dataPtr->LinkSDFString(eta.geomOrLightType);
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
      Entity parent = _ecm.EntityByComponents(
          components::Model(), components::Name(eta.parentName));
      if (parent == kNullEntity)
      {
        ignerr << "Unable to find " << eta.parentName << " in the ECM. "
               << std::endl;
         continue;
      }

      // generate unique link name
      // note passing components::Link() as arg to EntityByComponents causes
      // a crash on exit, see issue #1158
      std::string linkName = "link";
      Entity linkEnt = _ecm.EntityByComponents(
            /*components::Link(),*/ components::ParentEntity(parent),
            components::Name(linkName));
      int64_t counter = 0;
      while (linkEnt)
      {
        linkName = std::string("link") + "_" + std::to_string(++counter);
        linkEnt = _ecm.EntityByComponents(
            /*components::Link(),*/ components::ParentEntity(parent),
            components::Name(linkName));
      }

      linkSdf.SetName(linkName);
      auto entity = this->dataPtr->entityCreator->CreateEntities(&linkSdf);
      this->dataPtr->entityCreator->SetParent(entity, parent);

      // traverse the tree and add all new entities created by the entity creator
      // to the set
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
          event->ParentEntity());
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::LightSDFString(
    const std::string &_lightType) const
{
  std::stringstream lightStr;
  lightStr << "<light name='light' type='" << _lightType << "'>";

  if (_lightType == "directional")
  {
    lightStr
        << "<cast_shadows>false</cast_shadows>"
        << "<diffuse>1.0 1.0 1.0 1</diffuse>"
        << "<specular>0.5 0.5 0.5 1</specular>";
  }
  else if (_lightType == "spot")
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
  else if (_lightType == "point")
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
    ignwarn << "Light type not supported: " << _lightType << std::endl;
    return std::string();
  }

  lightStr << "</light>";
  return lightStr.str();
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::GeomSDFString(
    const std::string &_geomType) const
{
  math::Vector3d size = math::Vector3d::One;
  std::stringstream geomStr;
  geomStr << "<geometry>";
  if (_geomType == "box")
  {
    geomStr
      << "<box>"
      << "  <size>" << size << "</size>"
      << "</box>";
  }
  else if (_geomType == "sphere")
  {
    geomStr
      << "<sphere>"
      << "  <radius>" << size.X() * 0.5 << "</radius>"
      << "</sphere>";
  }
  else if (_geomType == "cylinder")
  {
    geomStr
      << "<cylinder>"
      << "  <radius>" << size.X() * 0.5 << "</radius>"
      << "  <length>" << size.Z() << "</length>"
      << "</cylinder>";
  }
  else if (_geomType == "capsule")
  {
    geomStr
      << "<capsule>"
      << "  <radius>" << size.X() * 0.5 << "</radius>"
      << "  <length>" << size.Z() << "</length>"
      << "</capsule>";
  }
  else if (_geomType == "ellipsoid")
  {
    geomStr
      << "<ellipsoid>"
      << "  <radii>" << size.X() * 0.5 << "</radii>"
      << "</ellipsoid>";
  }
  else
  {
    ignwarn << "Geometry type not supported: " << _geomType << std::endl;
    return std::string();
  }


  geomStr << "</geometry>";
  return geomStr.str();
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::LinkSDFString(
    const std::string &_geomOrLightType) const
{

  std::stringstream linkStr;
  if (_geomOrLightType == "empty")
  {
    linkStr << "<link name='link'/>";
    return linkStr.str();
  }

  std::string geomOrLightStr;
  if (_geomOrLightType == "spot" || _geomOrLightType == "directional" ||
      _geomOrLightType == "point")
  {
    geomOrLightStr = this->LightSDFString(_geomOrLightType);
    linkStr
        << "<link name='link'>"
        << geomOrLightStr
        << "</link>";
  }
  else
  {
    geomOrLightStr = this->GeomSDFString(_geomOrLightType);
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
sdf::Sensor ModelEditorPrivate::CreateSensor(const std::string &_type) const
{
  sdf::Sensor sensor;

  std::string type;

  // Replace spaces with underscores.
  common::replaceAll(type, _type, " ", "_");

  sensor.SetName("default");
  sensor.SetType(_type);
  if (type == "air_pressure")
  {
    sdf::AirPressure airpressure;
    sensor.SetAirPressureSensor(airpressure);
  }
  else if (type == "altimeter")
  {
    sdf::Altimeter altimeter;
    sensor.SetAltimeterSensor(altimeter);
  }
  else
  {
    ignerr << "Unable to create sensor type[" << _type << "]\n";
  }

  return sensor;
}

/////////////////////////////////////////////////
void ModelEditorPrivate::HandleAddEntity(const std::string &_geomOrLightType,
  const std::string &_type, const Entity _parentEntity)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::string entType = common::lowercase(_type);
  std::string geomLightType = common::lowercase(_geomOrLightType);

  EntityToAdd eta;
  eta.entityType = entType;
  eta.geomOrLightType = geomLightType;
  eta.parentEntity = _parentEntity;
  this->entitiesToAdd.push_back(eta);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ModelEditor,
                    ignition::gui::Plugin)
