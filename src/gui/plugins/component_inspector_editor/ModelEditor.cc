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

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>

#include <sdf/Box.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Mesh.hh>
#include <sdf/Sphere.hh>
#include <sdf/Link.hh>
#include <sdf/Sensor.hh>
#include <sdf/parser.hh>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Recreate.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/Util.hh"

#include "ModelEditor.hh"

namespace gz::sim
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
    /// \param[in] _parentEntity Parent entity
    /// \param[in] _data Additional variable data for specific instances
    public: void HandleAddEntity(const std::string &_geomOrLightType,
        const std::string &_entityType, Entity _parentEntity,
        const std::unordered_map<std::string, std::string> &_data);

    /// \brief Create a geom
    /// \param[in] _eta Entity to add.
    public: std::optional<sdf::Geometry> CreateGeom(
                const EntityToAdd &_eta) const;

    /// \brief Create a light
    /// \param[in] _eta Entity to add.
    public: std::optional<sdf::Light> CreateLight(
                const EntityToAdd &_eta) const;

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

    /// \brief Create a joint
    /// \param[in] _eta Entity to add.
    public: std::optional<sdf::Joint> CreateJoint(
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

using namespace gz;
using namespace sim;

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
  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void ModelEditor::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ModelEditor::Update");

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
      gzerr << "Parent entity not defined." << std::endl;
       continue;
    }

    if (eta.entityType == "link")
    {
      std::optional<sdf::Link> link = this->dataPtr->CreateLink(eta, _ecm);
      if (link)
      {
        Entity entity = this->dataPtr->entityCreator->CreateEntities(&(*link));
        this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
        // Make sure to mark the parent as needing recreation. This will
        // tell the server to rebuild the model with the new link.
        _ecm.CreateComponent(eta.parentEntity, components::Recreate());
        entities.push_back(entity);
      }
    }
    else if (eta.entityType == "sensor")
    {
      std::optional<sdf::Sensor> sensor =
        this->dataPtr->CreateSensor(eta, _ecm);
      if (sensor)
      {
        Entity entity = this->dataPtr->entityCreator->CreateEntities(
            &(*sensor));
        this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
        // Make sure to mark the parent as needing recreation. This will
        // tell the server to rebuild the model with the new link.
        _ecm.CreateComponent(topLevelModel(eta.parentEntity, _ecm),
                             components::Recreate());
        entities.push_back(entity);
      }
    }
    else if (eta.entityType == "joint")
    {
      std::optional<sdf::Joint> joint = this->dataPtr->CreateJoint(eta, _ecm);
      if (joint)
      {
        Entity entity = this->dataPtr->entityCreator->CreateEntities(
            &(*joint), true);
        this->dataPtr->entityCreator->SetParent(entity, eta.parentEntity);
        // Make sure to mark the parent as needing recreation. This will
        // tell the server to rebuild the model with the new link.
        _ecm.CreateComponent(eta.parentEntity, components::Recreate());
        entities.push_back(entity);
      }
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

  // use GuiNewRemovedEntities event to update other gui plugins
  // note this event will be removed in Gazebo Garden
  std::set<Entity> removedEntities;
  gz::sim::gui::events::GuiNewRemovedEntities event(
      newEntities, removedEntities);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &event);

  this->dataPtr->entitiesToAdd.clear();
}

/////////////////////////////////////////////////
bool ModelEditor::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == sim::gui::events::ModelEditorAddEntity::kType)
  {
    auto event = reinterpret_cast<gui::events::ModelEditorAddEntity *>(_event);
    if (event)
    {
      // Convert to an unordered map of STL strings for convenience
      std::unordered_map<std::string, std::string> data;
      for (auto key : event->Data().toStdMap())
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
std::optional<sdf::Light> ModelEditorPrivate::CreateLight(
    const EntityToAdd &_eta) const
{
  sdf::Light light;
  light.SetCastShadows(false);
  light.SetDiffuse(math::Color(1, 1, 1, 1));
  light.SetSpecular(math::Color(0.5, 0.5, 0.5, 0.5));

  if (_eta.geomOrLightType == "directional")
  {
    light.SetType(sdf::LightType::DIRECTIONAL);
  }
  else if (_eta.geomOrLightType == "spot" || _eta.geomOrLightType == "point")
  {
    light.SetType(sdf::LightType::SPOT);
    light.SetAttenuationRange(4);
    light.SetConstantAttenuationFactor(0.2);
    light.SetLinearAttenuationFactor(0.5);
    light.SetQuadraticAttenuationFactor(0.01);

    if (_eta.geomOrLightType == "spot")
    {
      light.SetSpotInnerAngle(0.1);
      light.SetSpotOuterAngle(0.5);
      light.SetSpotFalloff(0.8);
    }
  }
  else
  {
    gzwarn << "Light type not supported: "
      << _eta.geomOrLightType << std::endl;
    return std::nullopt;
  }

  return light;
}
/////////////////////////////////////////////////
std::optional<sdf::Geometry> ModelEditorPrivate::CreateGeom(
    const EntityToAdd &_eta) const
{
  math::Vector3d size = math::Vector3d::One;
  sdf::Geometry geom;

  if (_eta.geomOrLightType == "box")
  {
    sdf::Box shape;
    shape.SetSize(size);
    geom.SetBoxShape(shape);
    geom.SetType(sdf::GeometryType::BOX);
  }
  else if (_eta.geomOrLightType == "sphere")
  {
    sdf::Sphere shape;
    shape.SetRadius(size.X() * 0.5);
    geom.SetSphereShape(shape);
    geom.SetType(sdf::GeometryType::SPHERE);
  }
  else if (_eta.geomOrLightType == "cone")
  {
    sdf::Cone shape;
    shape.SetRadius(size.X() * 0.5);
    shape.SetLength(size.Z());
    geom.SetConeShape(shape);
    geom.SetType(sdf::GeometryType::CONE);
  }
  else if (_eta.geomOrLightType == "cylinder")
  {
    sdf::Cylinder shape;
    shape.SetRadius(size.X() * 0.5);
    shape.SetLength(size.Z());
    geom.SetCylinderShape(shape);
    geom.SetType(sdf::GeometryType::CYLINDER);
  }
  else if (_eta.geomOrLightType == "capsule")
  {
    sdf::Capsule shape;
    shape.SetRadius(size.X() * 0.5);
    shape.SetLength(size.Z());
    geom.SetCapsuleShape(shape);
    geom.SetType(sdf::GeometryType::CAPSULE);
  }
  else if (_eta.geomOrLightType == "ellipsoid")
  {
    sdf::Ellipsoid shape;
    shape.SetRadii(size * 0.5);
    geom.SetEllipsoidShape(shape);
    geom.SetType(sdf::GeometryType::ELLIPSOID);
  }
  else if (_eta.geomOrLightType == "mesh")
  {
    sdf::Mesh shape;
    shape.SetUri(_eta.data.at("uri"));
    geom.SetMeshShape(shape);
    geom.SetType(sdf::GeometryType::MESH);
  }
  else
  {
    gzwarn << "Geometry type not supported: "
      << _eta.geomOrLightType << std::endl;
    return std::nullopt;
  }

  return geom;
}

/////////////////////////////////////////////////
std::optional<sdf::Link> ModelEditorPrivate::CreateLink(
    const EntityToAdd &_eta, EntityComponentManager &_ecm) const
{
  sdf::Link link;
  if (_eta.parentEntity == kNullEntity)
  {
    gzerr << "Parent entity not defined." << std::endl;
    return std::nullopt;
  }

  if (_eta.geomOrLightType == "spot" || _eta.geomOrLightType == "directional" ||
      _eta.geomOrLightType == "point")
  {
    std::optional<sdf::Light> light = this->CreateLight(_eta);
    if (light)
      link.AddLight(*light);
  }
  else
  {
    std::optional<sdf::Geometry> geom = this->CreateGeom(_eta);
    if (geom)
    {
      sdf::Collision collision;
      collision.SetName("collision");
      collision.SetGeom(*geom);
      link.AddCollision(collision);

      sdf::Visual visual;
      visual.SetName("visual");
      visual.SetGeom(*geom);
      link.AddVisual(visual);
    }
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

  link.SetName(linkName);
  return link;
}

/////////////////////////////////////////////////
std::optional<sdf::Sensor> ModelEditorPrivate::CreateSensor(
    const EntityToAdd &_eta, EntityComponentManager &_ecm) const
{
  // Exit early if there is no parent entity
  if (_eta.parentEntity == kNullEntity)
  {
    gzerr << "Parent entity not defined." << std::endl;
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
std::optional<sdf::Joint> ModelEditorPrivate::CreateJoint(
    const EntityToAdd &_eta, EntityComponentManager &_ecm) const
{
  sdf::Joint joint;

  if (_eta.geomOrLightType == "ball")
    joint.SetType(sdf::JointType::BALL);
  else if (_eta.geomOrLightType == "continuous")
    joint.SetType(sdf::JointType::CONTINUOUS);
  else if (_eta.geomOrLightType == "fixed")
    joint.SetType(sdf::JointType::FIXED);
  else if (_eta.geomOrLightType == "gearbox")
    joint.SetType(sdf::JointType::GEARBOX);
  else if (_eta.geomOrLightType == "prismatic")
    joint.SetType(sdf::JointType::PRISMATIC);
  else if (_eta.geomOrLightType == "revolute")
    joint.SetType(sdf::JointType::REVOLUTE);
  else if (_eta.geomOrLightType == "revolute2")
    joint.SetType(sdf::JointType::REVOLUTE2);
  else if (_eta.geomOrLightType == "screw")
    joint.SetType(sdf::JointType::SCREW);
  else
  {
    gzwarn << "Joint type not supported: "
      << _eta.geomOrLightType << std::endl;

    return std::nullopt;
  }

  joint.SetParentName(_eta.data.at("parent_link"));
  joint.SetChildName(_eta.data.at("child_link"));

  std::string jointName = "joint";
  Entity jointEnt = _ecm.EntityByComponents(
      components::ParentEntity(_eta.parentEntity),
      components::Name(jointName));
  int64_t counter = 0;
  while (jointEnt)
  {
    jointName = std::string("joint") + "_" + std::to_string(++counter);
    jointEnt = _ecm.EntityByComponents(
        components::ParentEntity(_eta.parentEntity),
        components::Name(jointName));
  }

  joint.SetName(jointName);

  return joint;
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
