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
#include <regex>
#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <sdf/Link.hh>
#include <sdf/parser.hh>


#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ModelEditor.hh"

namespace ignition::gazebo
{
  class ModelEditorPrivate
  {
    public: void Initialize();

    public: void HandleAddEntity(const std::string &_geomOrLightType,
        const std::string &_entityType, const std::string &/*_parent*/);

    public: std::string GeomSDFString(
        const std::string &_geomType) const;

    public: std::string LightSDFString(
        const std::string &_geomType) const;


    public: std::string LinkSDFString(
        const std::string &_geomType) const;

    /// \brief Generate a unique entity id.
    /// \return The unique entity id
    // public: Entity UniqueId() const;

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene = nullptr;

    /// \brief Entity to add to the model editor
    public: std::string geomOrLightType;

    /// \brief Type of entity to add
    public: std::string entityType;

    /// \brief Parent entity to add the entity to
    public: std::string parentEntity;

    /// \brief Entity Creator API.
    public: std::unique_ptr<SdfEntityCreator> entityCreator{nullptr};

    /// \brief A record of the ids in the editor
    /// for easy deletion of visuals later
    public: std::vector<Entity> entityIds;

    /// \brief Mutex to protect the entity sdf list
    public: std::mutex mutex;

    /// \brief Links to add to the ECM
    public: std::vector<sdf::Link> linksToAdd;

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
    // create entities in ECM on the GUI side.
    // Note we have to start the entity id at an offset so it does not conflict
    // with the ones on the server. The log playback starts at max / 2
    // On the gui side, we will start entity id at an offset of max / 4
    // todo(anyone) set a better entity create offset
//    _ecm.SetEntityCreateOffset(math::MAX_I64 / 4);
    this->dataPtr->entityCreator = std::make_unique<SdfEntityCreator>(
        _ecm, this->dataPtr->eventMgr);
  }


  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // add link entities to the ECM
  std::set<Entity> newEntities;
  for (auto & linkSdf : this->dataPtr->linksToAdd)
  {
    Entity parent = _ecm.EntityByComponents(
        components::Model(), components::Name(this->dataPtr->parentEntity));
    if (parent == kNullEntity)
    {
      ignerr << "Unable to find " << this->dataPtr->parentEntity
             << " in the ECM. " << std::endl;
       continue;
    }

    // generate unique link name
    std::string linkName = "link";
    Entity linkEnt = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(parent),
          components::Name(linkName));
    int64_t counter = 0;
    while (linkEnt)
    {
      linkName = std::string("link") + "_" + std::to_string(++counter);
      linkEnt = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(parent),
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

  for (const auto & ent: newEntities)
      std::cerr << "creating entity in ecm " << ent << std::endl;

  // use tmp AddedRemovedEntities event to update other gui plugins
  // note this event will be removed in Ignition Garden
  std::set<Entity> removedEntities;
  ignition::gazebo::gui::events::AddedRemovedEntities event(
      newEntities, removedEntities);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &event);

  this->dataPtr->linksToAdd.clear();
}

/////////////////////////////////////////////////
bool ModelEditor::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gazebo::gui::events::ModelEditorAddEntity::kType)
  {
    auto event = reinterpret_cast<gui::events::ModelEditorAddEntity *>(_event);
    if (event)
    {
      this->dataPtr->geomOrLightType = event->Entity().toStdString();
      this->dataPtr->entityType = event->EntityType().toStdString();
      this->dataPtr->parentEntity = event->ParentEntity().toStdString();

      std::cerr << "model editor add entity event " << event->EntityType().toStdString()
                << " " << this->dataPtr->geomOrLightType << std::endl;

      this->dataPtr->HandleAddEntity(this->dataPtr->geomOrLightType,
          this->dataPtr->entityType,
          this->dataPtr->parentEntity);

    }
  }
  else if (_event->type() == ignition::gui::events::Render::kType)
  {
    // initialize rendering
    this->dataPtr->Initialize();

    // do something in rendering thread

  }


  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ModelEditorPrivate::Initialize()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }
//    this->sceneManager.SetScene(this->scene);
  }
}

/////////////////////////////////////////////////
//Entity ModelEditorPrivate::UniqueId()
//{
//  auto timeout = 100000u;
//  for (auto i = 0u; i < timeout; ++i)
//  {
//    Entity id = std::numeric_limits<uint64_t>::max() - i;
//    if (!this->sceneManager.HasEntity(id))
//      return id;
//  }
//  return kNullEntity;
//}

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
  if (_geomOrLightType == "spot" || geomOrLightType == "directional" ||
       geomOrLightType == "point")
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
void ModelEditorPrivate::HandleAddEntity(const std::string &_geomOrLightType,
  const std::string &_type, const std::string &/*_parentEntity*/)
{
  std::string entType = common::lowercase(_type);
  std::string geomLightType = common::lowercase(_geomOrLightType);
  if (entType == "link")
  {
    // auto model = this->scene->VisualByName(parentEntity);
    // if (!model)
    // {
    //   ignerr << "Unable to add link to model. "
    //          << "Parent entity: '" << parentEntity << "' not found. "
    //          << std::endl;
    // }

    // auto link = this->scene->CreateVisual();
    // auto visual = this->scene->CreateVisual();

    // rendering::GeometryPtr geom;
    // if (entity == "box")
    //   geom = this->scene->CreateBox();
    // else if (entity == "cylinder")
    //   geom = this->scene->CreateCylinder();
    // else if (entity == "sphere")
    //   geom = this->scene->CreateSphere();

    // visual->AddGeometry(geom);
    // link->AddChild(visual);
    // model->AddChild(link);

    // create an sdf::Link to it can be added to the ECM throught the
    // CreateEntities call
    std::string linkSDFStr = this->LinkSDFString(geomLightType);
    if (!linkSDFStr.empty())
    {
      linkSDFStr = std::string("<sdf version='") + SDF_VERSION + "'>" +
          linkSDFStr + "</sdf>";

      sdf::ElementPtr linkElem(new sdf::Element);
      sdf::initFile("link.sdf", linkElem);
      sdf::readString(linkSDFStr, linkElem);
      sdf::Link linkSdf;
      linkSdf.Load(linkElem);

      std::lock_guard<std::mutex> lock(this->mutex);
      this->linksToAdd.push_back(linkSdf);
    }
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ModelEditor,
                    ignition::gui::Plugin)
