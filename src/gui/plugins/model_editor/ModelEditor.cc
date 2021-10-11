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


#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ModelEditor.hh"

namespace ignition::gazebo
{
  class ModelEditorPrivate
  {
    public: void Initialize();

    public: void HandleAddEntity(const std::string &_entity,
        const std::string &_type, const std::string &_parent);

    /// \brief Generate a unique entity id.
    /// \return The unique entity id
    // public: Entity UniqueId() const;

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene = nullptr;

    /// \brief Entity to add to the model editor
    public: std::string entityToAdd;

    /// \brief Type of entity to add
    public: std::string entityType;

    /// \brief Parent entity to add the entity to
    public: std::string parentEntity;

    /// \brief True if there is an entity to be added to the editor
    public: bool addEntityDirty = false;

    /// \brief Scene manager
//    public: SceneManager sceneManager;

    /// \brief A record of the ids in the editor
    /// for easy deletion of visuals later
    public: std::vector<Entity> entityIds;
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
}

/////////////////////////////////////////////////
bool ModelEditor::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gazebo::gui::events::ModelEditorAddEntity::kType)
  {
    auto event = reinterpret_cast<gui::events::ModelEditorAddEntity *>(_event);
    if (event)
    {
      this->dataPtr->entityToAdd = event->Entity().toStdString();
      this->dataPtr->entityType = event->EntityType().toStdString();
      this->dataPtr->parentEntity = event->ParentEntity().toStdString();
      this->dataPtr->addEntityDirty = true;

      std::cerr << "model editor add entity event " << event->EntityType().toStdString() << " " << this->dataPtr->entityToAdd.toStdString() << std::endl;
    }
  }
  else if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->Initialize();
    if (this->dataPtr->addEntityDirty)
    {
      this->dataPtr->HandleAddEntity(this->dataPtr->entityToAdd,
          this->dataPtr->entityType,
          this->dataPtr->parentEntity);
      this->dataPtr->addEntityDirty = false;
    }
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
void ModelEditorPrivate::HandleAddEntity(const std::string &_entityToAdd,
  const std::string &_type, const std::string &_parentEntity)
{
  std::string type = common::lowercase(_type);
  std::string entity = common::lowercase(_entityToAdd);
  if (type == "link")
  {
    auto model = this->scene->VisualByName(parentEntity);
    if (!model)
    {
      ignerr << "Unable to add link to model. "
             << "Parent entity: '" << parentEntity << "' not found. "
             << std::endl;
    }

    auto link = this->scene->CreateVisual();
    auto visual = this->scene->CreateVisual();

    rendering::GeometryPtr geom;
    if (entity == "box")
      geom = this->scene->CreateBox();
    else if (entity == "cylinder")
      geom = this->scene->CreateCylinder();
    else if (entity == "sphere")
      geom = this->scene->CreateSphere();

    visual->AddGeometry(geom);
    link->AddChild(visual);
    model->AddChild(link);
  }
/*
  std::string entitySdfString = std::string(
      "<?xml version=\"1.0\"?>"
      "<sdf version=\"1.8\">"
        "<model name=\"template_model\">"
          "<pose>0 0 0.5 0 0 0</pose>"
          "<link name=\"link\">"
            "<visual name=\"visual\">"
              "<geometry>");
  if (_entity == "box")
  {
    entitySdfString += "<box>"
                         "<size>1 1 1</size>"
                       "</box>"
  }
  else if (_entity == "cylinder")
  {
    entitySdfString += "<cylinder>"
                         "<radius>0.5</radius>"
                         "<length>1.0</length>"
                       "</cylinder>"
  }
  else if (_entity == "sphere")
  {
    entitySdfString += "<sphere>"
                         "<radius>0.5</radius>"
                       "</sphere>"
  }
  entitySdfString +=
              "</geometry>"
            "</visual>"
          "</link>"
        "</model>"
      "</sdf>";


  sdf::Root root;
  root.LoadSdfString(entitySdfString);

  // create model
  Entity modelId = this->UniqueId();
  if (!modelId)
  {
    ignerr << "unable to generate unique Id" << std::endl;
    return;
  }

  sdf::Model model = *(root.Model());
//  model.SetName(common::Uuid().String());
//  auto model = this->sceneManager.CreateModel(
//      modelId, model, this->sceneManager.WorldId());
//  this->entityIds.push_back(modelId);

  // create link
  sdf::Link link = *(model.LinkByIndex(0));
  link.SetName(common::Uuid().String());
  Entity linkId = this->UniqueId();
  if (!linkId)
  {
    ignerr << "unable to generate unique Id" << std::endl;
    return;
  }
  this->sceneManager.CreateLink(linkId, link, modelId);
  this->entityIds.push_back(linkId);

  // create visual
  sdf::Visual visual = *(link.VisualByIndex(0));
  visual.SetName(common::Uuid().String());
  Entity visualId = this->UniqueId();
  if (!visualId)
  {
    ignerr << "unable to generate unique Id" << std::endl;
    return;
  }

  this->sceneManager.CreateVisual(visualId, visual, linkId);
  this->entityIds.push_back(visualId);
*/

}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ModelEditor,
                    ignition::gui::Plugin)
