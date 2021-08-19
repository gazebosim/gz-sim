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

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <algorithm>
#include <limits>
#include <iostream>
#include <string>
#include <vector>

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Uuid.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/Helpers.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/math/Vector2.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/Visual.hh>
#include <ignition/rendering/Scene.hh>

#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include <sdf/Root.hh>

#include "ignition/gazebo/rendering/RenderUtil.hh"
#include "ignition/gazebo/rendering/SceneManager.hh"

namespace ignition::gazebo
{
  class SpawnPrivate
  {
    /// \brief Update the 3D scene with new entities
    public: void OnRender();

    /// \brief Delete the visuals generated while an entity is being spawned.
    public: void TerminateSpawnPreview();

    /// \brief Generate a preview of a resource.
    /// \param[in] _sdf The SDF to be previewed.
    /// \return True on success, false if failure
    public: bool GeneratePreview(const sdf::Root &_sdf);

    /// \brief Handle model placement requests
    public: void HandleModelPlacement();

    /// \brief Retrieve the point on a plane at z = 0 in the 3D scene hit by a
    /// ray cast from the given 2D screen coordinates.
    /// \param[in] _screenPos 2D coordinates on the screen, in pixels.
    /// \param[in] _camera User camera
    /// \param[in] _rayQuery Ray query for mouse clicks
    /// \param[in] _offset Offset along the plane normal
    /// \return 3D coordinates of a point in the 3D scene.
    math::Vector3d ScreenToPlane(
      const ignition::math::Vector2i &_screenPos,
      const ignition::rendering::CameraPtr &_camera,
      const ignition::rendering::RayQueryPtr &_rayQuery,
      const float offset = 0.0);

    /// \brief Generate a unique entity id.
    /// \return The unique entity id
    ignition::gazebo::Entity UniqueId();

    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Flag for indicating whether we are spawning or not.
    public: bool isSpawning = false;

    /// \brief Flag for indicating whether the user is currently placing a
    /// resource with the shapes plugin or not
    public: bool isPlacing = false;

    /// \brief The SDF string of the resource to be used with plugins that spawn
    /// entities.
    public: std::string spawnSdfString;

    /// \brief Path of an SDF file, to be used with plugins that spawn entities.
    public: std::string spawnSdfPath;

    /// \brief Pointer to the rendering scene
    public: ignition::rendering::ScenePtr scene{nullptr};

    /// \brief A record of the ids currently used by the entity spawner
    /// for easy deletion of visuals later
    public: std::vector<ignition::gazebo::Entity> previewIds;

    /// \brief The visual generated from the spawnSdfString
    public: ignition::rendering::NodePtr spawnPreview{nullptr};

    /// \brief Scene manager
    public: ignition::gazebo::SceneManager sceneManager;

    /// \brief The pose of the spawn preview.
    public: ignition::math::Pose3d spawnPreviewPose =
            ignition::math::Pose3d::Zero;

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
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
Spawn::Spawn()
  : ignition::gui::Plugin(),
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

  // World name from window, to construct default topics and services
  auto worldNames = gui::worldNames();
  if (!worldNames.empty())
    this->dataPtr->worldName = worldNames[0].toStdString();

  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);
}


// TODO(ahcorde): Replace this when this PR is on ign-rendering6
/////////////////////////////////////////////////
math::Vector3d SpawnPrivate::ScreenToPlane(
    const math::Vector2i &_screenPos,
    const rendering::CameraPtr &_camera,
    const rendering::RayQueryPtr &_rayQuery,
    const float offset)
{
  // Normalize point on the image
  double width = _camera->ImageWidth();
  double height = _camera->ImageHeight();

  double nx = 2.0 * _screenPos.X() / width - 1.0;
  double ny = 1.0 - 2.0 * _screenPos.Y() / height;

  // Make a ray query
  _rayQuery->SetFromCamera(
      _camera, math::Vector2d(nx, ny));

  ignition::math::Planed plane(ignition::math::Vector3d(0, 0, 1), offset);

  math::Vector3d origin = _rayQuery->Origin();
  math::Vector3d direction = _rayQuery->Direction();
  double distance = plane.Distance(origin, direction);
  return origin + direction * distance;
}

/////////////////////////////////////////////////
void SpawnPrivate::HandleModelPlacement()
{
  if (!this->isPlacing)
    return;

  if (this->spawnPreview && this->hoverDirty)
  {
    math::Vector3d pos = this->ScreenToPlane(
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

    math::Pose3d modelPose = this->spawnPreviewPose;
    std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
        [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
    {
      if (!_result)
        ignerr << "Error creating model" << std::endl;
    };
    math::Vector3d pos = this->ScreenToPlane(
      this->mouseEvent.Pos(), this->camera, this->rayQuery);
    pos.Z(modelPose.Pos().Z());
    msgs::EntityFactory req;
    if (!this->spawnSdfString.empty())
    {
      req.set_sdf(this->spawnSdfString);
    }
    else if (!this->spawnSdfPath.empty())
    {
      req.set_sdf_filename(this->spawnSdfPath);
    }
    else
    {
      ignwarn << "Failed to find SDF string or file path" << std::endl;
    }
    req.set_allow_renaming(true);
    msgs::Set(req.mutable_pose(), math::Pose3d(pos, modelPose.Rot()));

    if (this->createCmdService.empty())
    {
      this->createCmdService = "/world/" + this->worldName
          + "/create";
    }
    this->createCmdService = transport::TopicUtils::AsValidTopic(
        this->createCmdService);
    if (this->createCmdService.empty())
    {
      ignerr << "Failed to create valid create command service for world ["
             << this->worldName <<"]" << std::endl;
      return;
    }

    this->node.Request(this->createCmdService, req, cb);
    this->isPlacing = false;
    this->mouseDirty = false;
    this->spawnSdfString.clear();
    this->spawnSdfPath.clear();
  }
}

/////////////////////////////////////////////////
ignition::gazebo::Entity SpawnPrivate::UniqueId()
{
  auto timeout = 100000u;
  for (auto i = 0u; i < timeout; ++i)
  {
    Entity id = std::numeric_limits<uint64_t>::max() - i;
    if (!this->sceneManager.HasEntity(id))
      return id;
  }
  return kNullEntity;
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
      if (cam)
      {
        if (std::get<bool>(cam->UserData("user-camera")))
        {
          this->camera = cam;

          // Ray Query
          this->rayQuery = this->camera->Scene()->CreateRayQuery();

          igndbg << "Spawn plugin is using camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }
  }

  // Spawn
  IGN_PROFILE("IgnRenderer::Render Spawn");
  if (this->isSpawning)
  {
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
    else
    {
      ignwarn << "Failed to spawn: no SDF string or path" << std::endl;
    }
    this->isPlacing = this->GeneratePreview(root);
    this->isSpawning = false;
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

  this->HandleModelPlacement();
}

/////////////////////////////////////////////////
void SpawnPrivate::TerminateSpawnPreview()
{
  for (auto _id : this->previewIds)
    this->sceneManager.RemoveEntity(_id);
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
    ignwarn << "Only model entities can be spawned at the moment." << std::endl;
    this->TerminateSpawnPreview();
    return false;
  }

  if (_sdf.Model())
  {
    // Only preview first model
    sdf::Model model = *(_sdf.Model());
    this->spawnPreviewPose = model.RawPose();
    model.SetName(ignition::common::Uuid().String());
    Entity modelId = this->UniqueId();
    if (!modelId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->spawnPreview =
      this->sceneManager.CreateModel(
          modelId, model,
          this->sceneManager.WorldId());

    this->previewIds.push_back(modelId);
    for (auto j = 0u; j < model.LinkCount(); j++)
    {
      sdf::Link link = *(model.LinkByIndex(j));
      link.SetName(ignition::common::Uuid().String());
      Entity linkId = this->UniqueId();
      if (!linkId)
      {
        this->TerminateSpawnPreview();
        return false;
      }
      this->sceneManager.CreateLink(
          linkId, link, modelId);
      this->previewIds.push_back(linkId);
      for (auto k = 0u; k < link.VisualCount(); k++)
      {
       sdf::Visual visual = *(link.VisualByIndex(k));
       visual.SetName(ignition::common::Uuid().String());
       Entity visualId = this->UniqueId();
       if (!visualId)
       {
         this->TerminateSpawnPreview();
         return false;
       }
       this->sceneManager.CreateVisual(
           visualId, visual, linkId);
       this->previewIds.push_back(visualId);
      }
    }
  }
  else if (_sdf.Light())
  {
    // Only preview first model
    sdf::Light light = *(_sdf.Light());
    this->spawnPreviewPose = light.RawPose();
    light.SetName(ignition::common::Uuid().String());
    Entity lightVisualId = this->UniqueId();
    if (!lightVisualId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    Entity lightId = this->UniqueId();
    if (!lightId)
    {
      this->TerminateSpawnPreview();
      return false;
    }
    this->spawnPreview =
      this->sceneManager.CreateLight(
          lightId, light,
          this->sceneManager.WorldId());
    this->sceneManager.CreateLightVisual(
        lightVisualId, light, lightId);

    this->previewIds.push_back(lightId);
    this->previewIds.push_back(lightVisualId);
  }
  return true;
}

////////////////////////////////////////////////
bool Spawn::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  else if (_event->type() == ignition::gui::events::LeftClickOnScene::kType)
  {
    ignition::gui::events::LeftClickOnScene *_e =
      static_cast<ignition::gui::events::LeftClickOnScene*>(_event);
    this->dataPtr->mouseEvent = _e->Mouse();
    this->dataPtr->mouseDirty = true;
  }
  else if (_event->type() == ignition::gui::events::HoverOnScene::kType)
  {
    ignition::gui::events::HoverOnScene *_e =
      static_cast<ignition::gui::events::HoverOnScene*>(_event);
    this->dataPtr->mouseHoverPos = _e->Mouse().Pos();
    this->dataPtr->hoverDirty = true;
  }
  else if (_event->type() ==
    ignition::gui::events::SpawnFromDescription::kType)
  {
    ignition::gui::events::SpawnFromDescription *_e =
      static_cast<ignition::gui::events::SpawnFromDescription*>(_event);
    this->dataPtr->spawnSdfString = _e->Description();
    this->dataPtr->isSpawning = true;
  }
  else if (_event->type() == ignition::gui::events::SpawnFromPath::kType)
  {
    auto spawnPreviewPathEvent =
      reinterpret_cast<ignition::gui::events::SpawnFromPath *>(_event);
    this->dataPtr->spawnSdfPath = spawnPreviewPathEvent->FilePath();
    this->dataPtr->isSpawning = true;
  }
  else if (_event->type() == ignition::gui::events::KeyReleaseOnScene::kType)
  {
    ignition::gui::events::KeyReleaseOnScene *_e =
      static_cast<ignition::gui::events::KeyReleaseOnScene*>(_event);
    if (_e->Key().Key() == Qt::Key_Escape)
    {
      this->dataPtr->escapeReleased = true;
    }
  }

  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Spawn,
                    ignition::gui::Plugin)
