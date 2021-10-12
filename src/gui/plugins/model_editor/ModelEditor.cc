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
        const std::string &_geomType,
        const math::Vector3d &_size = math::Vector3d::One) const;

    public: std::string LinkSDFString(
        const std::string &_geomType,
        const math::Vector3d &_size = math::Vector3d::One) const;

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
    _ecm.SetEntityCreateOffset(math::MAX_I64 / 4);
    this->dataPtr->entityCreator = std::make_unique<SdfEntityCreator>(
        _ecm, this->dataPtr->eventMgr);
  }


  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // add link entities to the ECM
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

    // generate link name
    std::string linkName = "link";
    Entity linkEnt = _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(parent),
          components::Name(linkName));
    int64_t counter = 0;
    while (linkEnt)
    {
      linkName = std::string("link") + "_" + std::to_string(++counter);
      _ecm.EntityByComponents(
          components::Link(), components::ParentEntity(parent),
          components::Name(linkName));
    }

    std::cerr << "creating entity in ecm " << linkName << std::endl;
    linkSdf.SetName(linkName);
    auto entity = this->dataPtr->entityCreator->CreateEntities(&linkSdf);
    this->dataPtr->entityCreator->SetParent(entity, parent);
  }
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
std::string ModelEditorPrivate::GeomSDFString(
    const std::string &_geomType,
    const math::Vector3d &_size) const
{
  std::stringstream geomStr;
  geomStr << "<geometry>";
  if (_geomType == "box")
  {
    geomStr
      << "<box>"
      << "  <size>" << _size << "</size>"
      << "</box>";
  }
  else if (_geomType == "sphere")
  {
    geomStr
      << "<sphere>"
      << "  <radius>" << _size.X() * 0.5 << "</radius>"
      << "</sphere>";
  }
  else if (_geomType == "cylinder")
  {
    geomStr
      << "<cylinder>"
      << "  <radius>" << _size.X() * 0.5 << "</radius>"
      << "  <length>" << _size.Z() << "</length>"
      << "</cylinder>";
  }
  geomStr << "</geometry>";
  return geomStr.str();
}

/////////////////////////////////////////////////
std::string ModelEditorPrivate::LinkSDFString(
    const std::string &_geomType,
    const math::Vector3d &_size) const
{
  std::string geomStr = this->GeomSDFString(_geomType, _size);
  std::stringstream linkStr;
  linkStr
      << "<sdf version='" << SDF_VERSION <<"'>"
      << "  <link name='link'>"
      << "  <visual name='visual'>"
      << geomStr
      << "  </visual>"
      << "  <collision name='collision'>"
      << geomStr
      << "  </collision>"
      << "  </link>"
      << "</sdf>";

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
    sdf::ElementPtr linkElem(new sdf::Element);
    sdf::initFile("link.sdf", linkElem);
    sdf::readString(this->LinkSDFString(geomLightType), linkElem);
    // std::cerr  << this->LinkSDFString("new_entity", geomLightType) << std::endl;
    sdf::Link linkSdf;
    linkSdf.Load(linkElem);

    std::lock_guard<std::mutex> lock(this->mutex);
    this->linksToAdd.push_back(linkSdf);
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ModelEditor,
                    ignition::gui::Plugin)
