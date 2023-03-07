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

#include "Spawn.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>

#include <algorithm>
#include <limits>
#include <iostream>
#include <string>
#include <vector>
#include <QQmlProperty>

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Uuid.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/math/Vector2.hh>
#include <gz/msgs/Utility.hh>

#include <gz/plugin/Register.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Utils.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Scene.hh>

#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include <sdf/Root.hh>

#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/rendering/SceneManager.hh"

namespace gz::sim
{
  class SpawnPrivate
  {
    /// \brief Perform operations in the render thread.
    public: void OnRender();

    /// \brief Delete the visuals generated while an entity is being spawned.
    public: void TerminateSpawnPreview();

    /// \brief Generate a preview of a resource.
    /// \param[in] _sdf The SDF to be previewed.
    /// \return True on success, false if failure
    public: bool GeneratePreview(const sdf::Root &_sdf);

    /// \brief Generate a preview of a resource.
    /// \param[in] _sdf The name of the resource to be previewed.
    /// \return True on success, false if failure
    public: bool GeneratePreview(const std::string &_name);

    /// \brief Handle placement requests
    public: void HandlePlacement();

    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief Flag for indicating whether the preview needs to be generated.
    public: bool generatePreview = false;

    /// \brief Flag for indicating whether the user is currently placing a
    /// resource or not
    public: bool isPlacing = false;

    /// \brief The SDF string of the resource to be used with plugins that spawn
    /// entities.
    public: std::string spawnSdfString;

    /// \brief Path of an SDF file, to be used with plugins that spawn entities.
    public: std::string spawnSdfPath;

    /// \brief The name of a resource to clone
    public: std::string spawnCloneName;

    /// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene{nullptr};

    /// \brief A record of the ids currently used by the entity spawner
    /// for easy deletion of visuals later
    public: std::vector<Entity> previewIds;

    /// \brief Pointer to the preview that the user is placing.
    public: rendering::NodePtr spawnPreview{nullptr};

    /// \brief Scene manager
    public: SceneManager sceneManager;

    /// \brief The pose of the spawn preview.
    public: math::Pose3d spawnPreviewPose =
            math::Pose3d::Zero;

    /// \brief Mouse event
    public: common::MouseEvent mouseEvent;

    /// \brief Flag to indicate if mouse event is dirty
    public: bool mouseDirty = false;

    /// \brief Flag to indicate if hover event is dirty
    public: bool hoverDirty = false;

    /// \brief Flag to indicate whether the escape key has been released.
    public: bool escapeReleased = false;

    /// \brief The currently hovered mouse position in screen coordinates
    public: math::Vector2i mouseHoverPos = math::Vector2i::Zero;

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera{nullptr};

    /// \brief Name of service for creating entity
    public: std::string createCmdService;

    /// \brief Name of the world
    public: std::string worldName;

    /// \brief Text for popup error message
    public: QString errorPopupText;

    /// \brief Adds new line after each nChar.
    public: std::string AddNewLine(const std::string &_str, int _nChar);
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Spawn::Spawn()
  : gz::gui::Plugin(),
  dataPtr(std::make_unique<SpawnPrivate>())
{
}

/////////////////////////////////////////////////
Spawn::~Spawn() = default;

/////////////////////////////////////////////////
void Spawn::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Spawn";

  static bool done{false};
  if (done)
  {
    std::string msg{"Only one Spawn plugin is supported at a time."};
    gzerr << msg << std::endl;
    QQmlProperty::write(this->PluginItem(), "message",
        QString::fromStdString(msg));
    return;
  }
  done = true;

  // World name from window, to construct default topics and services
  auto worldNames = gui::worldNames();
  if (!worldNames.empty())
    this->dataPtr->worldName = worldNames[0].toStdString();

  gz::gui::App()->findChild
    <gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
std::string SpawnPrivate::AddNewLine(const std::string &_str, int _nChar)
{
  std::string out;
  out.reserve(_str.size() + _str.size() / _nChar);
  for (std::string::size_type i = 0; i < _str.size(); i++)
  {
    if (!(i % _nChar) && i)
    {
      out.append("-\n");
    }
    out.push_back(_str[i]);
  }
  return out;
}

/////////////////////////////////////////////////
void SpawnPrivate::HandlePlacement()
{
  if (!this->isPlacing)
    return;

  if (this->spawnPreview && this->hoverDirty)
  {
    math::Vector3d pos = gz::rendering::screenToPlane(
      this->mouseHoverPos, this->camera, this->rayQuery);
    pos.Z(this->spawnPreview->WorldPosition().Z());
    this->spawnPreview->SetWorldPosition(pos);
    this->hoverDirty = false;
  }
  if (this->mouseEvent.Button() == common::MouseEvent::LEFT &&
      this->mouseEvent.Type() == common::MouseEvent::RELEASE &&
      !this->mouseEvent.Dragging() && this->mouseDirty)
  {
    // Delete the generated visuals
    this->TerminateSpawnPreview();

    auto pose = this->spawnPreviewPose;
    std::function<void(const msgs::Boolean &, const bool)> cb =
        [](const msgs::Boolean &/*_rep*/, const bool _result)
    {
      if (!_result)
        gzerr << "Error creating entity" << std::endl;
    };
    math::Vector3d pos = gz::rendering::screenToPlane(
      this->mouseEvent.Pos(), this->camera, this->rayQuery);
    pos.Z(pose.Pos().Z());
    msgs::EntityFactory req;
    if (!this->spawnSdfString.empty())
    {
      req.set_sdf(this->spawnSdfString);
    }
    else if (!this->spawnSdfPath.empty())
    {
      req.set_sdf_filename(this->spawnSdfPath);
    }
    else if (!this->spawnCloneName.empty())
    {
      req.set_clone_name(this->spawnCloneName);
    }
    else
    {
      gzwarn << "Failed to find SDF string or file path" << std::endl;
    }
    req.set_allow_renaming(true);
    msgs::Set(req.mutable_pose(), math::Pose3d(pos, pose.Rot()));

    if (this->createCmdService.empty())
    {
      this->createCmdService = "/world/" + this->worldName
          + "/create";
    }
    this->createCmdService = transport::TopicUtils::AsValidTopic(
        this->createCmdService);
    if (this->createCmdService.empty())
    {
      gzerr << "Failed to create valid create command service for world ["
             << this->worldName <<"]" << std::endl;
      return;
    }

    this->node.Request(this->createCmdService, req, cb);
    this->isPlacing = false;
    this->mouseDirty = false;
    this->spawnSdfString.clear();
    this->spawnSdfPath.clear();
    this->spawnCloneName.clear();
  }
}

/////////////////////////////////////////////////
void SpawnPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }
    this->sceneManager.SetScene(this->scene);

    for (unsigned int i = 0; i < this->scene->NodeCount(); ++i)
    {
      auto cam = std::dynamic_pointer_cast<rendering::Camera>(
        this->scene->NodeByIndex(i));
      if (cam && cam->HasUserData("user-camera") &&
          std::get<bool>(cam->UserData("user-camera")))
      {
        this->camera = cam;

        // Ray Query
        this->rayQuery = this->camera->Scene()->CreateRayQuery();

        gzdbg << "Spawn plugin is using camera ["
               << this->camera->Name() << "]" << std::endl;
        break;
      }
    }
  }

  // Spawn
  GZ_PROFILE("GzRenderer::Render Spawn");
  if (this->generatePreview)
  {
    bool cloningResource = false;

    // Generate spawn preview
    rendering::VisualPtr rootVis = this->scene->RootVisual();
    sdf::Root root;
    if (!this->spawnSdfString.empty())
    {
      root.LoadSdfString(this->spawnSdfString);
    }
    else if (!this->spawnSdfPath.empty())
    {
      root.Load(this->spawnSdfPath);
    }
    else if (!this->spawnCloneName.empty())
    {
      this->isPlacing = this->GeneratePreview(this->spawnCloneName);
      cloningResource = true;
    }
    else
    {
      gzwarn << "Failed to spawn: no SDF string, path, or name of resource "
              << "to clone" << std::endl;
    }

    if (!cloningResource)
      this->isPlacing = this->GeneratePreview(root);

    this->generatePreview = false;
  }

  // Escape action, clear all selections and terminate any
  // spawned previews if escape button is released
  {
    if (this->escapeReleased)
    {
      this->TerminateSpawnPreview();
      this->escapeReleased = false;
    }
  }

  this->HandlePlacement();
}

/////////////////////////////////////////////////
void SpawnPrivate::TerminateSpawnPreview()
{
  for (auto _id : this->previewIds)
  {
    this->sceneManager.RemoveEntity(_id);
  }
  this->previewIds.clear();
  this->isPlacing = false;
}

/////////////////////////////////////////////////
bool SpawnPrivate::GeneratePreview(const sdf::Root &_sdf)
{
  // Terminate any pre-existing spawned entities
  this->TerminateSpawnPreview();

  if (nullptr == _sdf.Model() && nullptr == _sdf.Light())
  {
    gzwarn << "Only model or light entities can be spawned at the moment."
            << std::endl;
    return false;
  }

  if (_sdf.Model())
  {
    // Only preview first model
    sdf::Model model = *(_sdf.Model());
    this->spawnPreviewPose = model.RawPose();
    model.SetName(common::Uuid().String());
    Entity modelId = this->sceneManager.UniqueId();
    if (kNullEntity == modelId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->spawnPreview = this->sceneManager.CreateModel(
          modelId, model, this->sceneManager.WorldId());

    this->previewIds.push_back(modelId);
    for (auto j = 0u; j < model.LinkCount(); j++)
    {
      sdf::Link link = *(model.LinkByIndex(j));
      link.SetName(common::Uuid().String());
      Entity linkId = this->sceneManager.UniqueId();
      if (!linkId)
      {
        this->TerminateSpawnPreview();
        return false;
      }
      this->sceneManager.CreateLink(linkId, link, modelId);
      this->previewIds.push_back(linkId);
      for (auto k = 0u; k < link.VisualCount(); k++)
      {
        sdf::Visual visual = *(link.VisualByIndex(k));
        visual.SetName(common::Uuid().String());
        Entity visualId = this->sceneManager.UniqueId();
        if (!visualId)
        {
          this->TerminateSpawnPreview();
          return false;
        }
        this->sceneManager.CreateVisual(visualId, visual, linkId);
        this->previewIds.push_back(visualId);
      }
    }
  }
  else if (_sdf.Light())
  {
    // Only preview first light
    sdf::Light light = *(_sdf.Light());
    this->spawnPreviewPose = light.RawPose();
    light.SetName(common::Uuid().String());
    Entity lightVisualId = this->sceneManager.UniqueId();
    if (!lightVisualId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    Entity lightId = this->sceneManager.UniqueId();
    if (!lightId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->spawnPreview = this->sceneManager.CreateLight(
          lightId, light, light.Name(), this->sceneManager.WorldId());
    this->sceneManager.CreateLightVisual(
        lightVisualId, light, light.Name(), lightId);

    this->previewIds.push_back(lightId);
    this->previewIds.push_back(lightVisualId);
  }
  return true;
}

/////////////////////////////////////////////////
bool SpawnPrivate::GeneratePreview(const std::string &_name)
{
  // Terminate any pre-existing spawned entities
  this->TerminateSpawnPreview();

  Entity visualId = this->sceneManager.UniqueId();
  if (!visualId)
  {
    this->TerminateSpawnPreview();
    return false;
  }

  auto visualChildrenPair = this->sceneManager.CopyVisual(visualId, _name,
      this->sceneManager.WorldId());
  if (!visualChildrenPair.first)
  {
    gzerr << "Copying a visual named " << _name << "failed.\n";
    return false;
  }

  this->spawnPreview = visualChildrenPair.first;
  this->spawnPreviewPose = this->spawnPreview->WorldPose();

  // save the copied chiled IDs before saving the copied parent visual ID in
  // order to ensure that the child visuals get deleted before the parent visual
  // (since the SceneManager::RemoveEntity call in this->TerminateSpawnPreview()
  // isn't recursive, deleting the parent visual before the child visuals could
  // result in dangling child visuals)
  const auto &visualChildIds = visualChildrenPair.second;
  for (auto reverse_it = visualChildIds.rbegin();
      reverse_it != visualChildIds.rend(); ++reverse_it)
    this->previewIds.push_back(*reverse_it);
  this->previewIds.push_back(visualId);

  return true;
}

////////////////////////////////////////////////
bool Spawn::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  else if (_event->type() == gz::gui::events::LeftClickOnScene::kType)
  {
    gz::gui::events::LeftClickOnScene *_e =
      static_cast<gz::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();
    if (this->dataPtr->generatePreview || this->dataPtr->isPlacing)
      this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == gz::gui::events::HoverOnScene::kType)
  {
    gz::gui::events::HoverOnScene *_e =
      static_cast<gz::gui::events::HoverOnScene*>(_event);
    this->dataPtr->mouseHoverPos = _e->Mouse().Pos();
    this->dataPtr->hoverDirty = true;
  }
  else if (_event->type() ==
    gz::gui::events::SpawnFromDescription::kType)
  {
    gz::gui::events::SpawnFromDescription *_e =
      static_cast<gz::gui::events::SpawnFromDescription*>(_event);
    this->dataPtr->spawnSdfString = _e->Description();
    this->dataPtr->generatePreview = true;
  }
  else if (_event->type() == gz::gui::events::SpawnFromPath::kType)
  {
    auto spawnPreviewPathEvent =
      reinterpret_cast<gz::gui::events::SpawnFromPath *>(_event);
    this->dataPtr->spawnSdfPath = spawnPreviewPathEvent->FilePath();
    this->dataPtr->generatePreview = true;
  }
  else if (_event->type() == gz::gui::events::SpawnCloneFromName::kType)
  {
    auto spawnCloneEvent =
      reinterpret_cast<gz::gui::events::SpawnCloneFromName *>(_event);
    if (spawnCloneEvent)
    {
      this->dataPtr->spawnCloneName = spawnCloneEvent->Name();
      this->dataPtr->generatePreview = true;
    }
  }
  else if (_event->type() == gz::gui::events::KeyReleaseOnScene::kType)
  {
    gz::gui::events::KeyReleaseOnScene *_e =
      static_cast<gz::gui::events::KeyReleaseOnScene*>(_event);
    if (_e->Key().Key() == Qt::Key_Escape)
    {
      this->dataPtr->escapeReleased = true;
    }
  }
  else if (_event->type() == gz::gui::events::DropOnScene::kType)
  {
    auto dropOnSceneEvent =
      reinterpret_cast<gz::gui::events::DropOnScene *>(_event);
    if (dropOnSceneEvent)
    {
      this->OnDropped(dropOnSceneEvent);
    }
  }

  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void Spawn::OnDropped(const gz::gui::events::DropOnScene *_event)
{
  if (nullptr == _event || nullptr == this->dataPtr->camera ||
      nullptr == this->dataPtr->rayQuery)
  {
    return;
  }

  if (_event->DropText().empty())
  {
    this->SetErrorPopupText("Dropped empty entity URI.");
    return;
  }

  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
      [](const gz::msgs::Boolean &_res, const bool _result)
  {
    if (!_result || !_res.data())
      gzerr << "Error creating dropped entity." << std::endl;
  };

  math::Vector3d pos = gz::rendering::screenToScene(
    _event->Mouse(),
    this->dataPtr->camera,
    this->dataPtr->rayQuery);

  msgs::EntityFactory req;
  std::string dropStr = _event->DropText();

  // Local meshes
  if (QUrl(QString::fromStdString(dropStr)).isLocalFile())
  {
    // mesh to sdf model
    common::rtrim(dropStr);

    if (!common::MeshManager::Instance()->IsValidFilename(dropStr))
    {
      std::string fixedDropStr = this->dataPtr->AddNewLine(dropStr, 55);
      std::string errTxt = "Invalid URI: " + fixedDropStr +
        "\nOnly Fuel URLs or mesh file types DAE, FBX, GLTF, OBJ, and STL\n"
        "are supported.";
      QString QErrTxt = QString::fromStdString(errTxt);
      this->SetErrorPopupText(QErrTxt);
      return;
    }

    // Fixes whitespace
    dropStr = common::replaceAll(dropStr, "%20", " ");

    std::string filename = common::basename(dropStr);
    std::vector<std::string> splitName = common::split(filename, ".");

    std::string sdf = "<?xml version='1.0'?>"
      "<sdf version='" + std::string(SDF_PROTOCOL_VERSION) + "'>"
        "<model name='" + splitName[0] + "'>"
          "<link name='link'>"
            "<visual name='visual'>"
              "<geometry>"
                "<mesh>"
                  "<uri>" + dropStr + "</uri>"
                "</mesh>"
              "</geometry>"
            "</visual>"
            "<collision name='collision'>"
              "<geometry>"
                "<mesh>"
                  "<uri>" + dropStr + "</uri>"
                "</mesh>"
              "</geometry>"
            "</collision>"
          "</link>"
        "</model>"
      "</sdf>";

    req.set_sdf(sdf);
  }
  // Resource from fuel
  else
  {
    req.set_sdf_filename(dropStr);
  }

  req.set_allow_renaming(true);
  msgs::Set(req.mutable_pose(),
      math::Pose3d(pos.X(), pos.Y(), pos.Z(), 1, 0, 0, 0));

  this->dataPtr->node.Request("/world/" + this->dataPtr->worldName + "/create",
      req, cb);
}

/////////////////////////////////////////////////
QString Spawn::ErrorPopupText() const
{
  return this->dataPtr->errorPopupText;
}

/////////////////////////////////////////////////
void Spawn::SetErrorPopupText(const QString &_errorTxt)
{
  this->dataPtr->errorPopupText = _errorTxt;
  this->ErrorPopupTextChanged();
  this->popupError();
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::Spawn,
                    gz::gui::Plugin)
