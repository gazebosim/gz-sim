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

#include "Lights.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <string>

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
  class LightsPrivate
  {
    /// \brief Handle shapes
    public: void handleLights();

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

    /// \brief Enable legacy features for plugin to work with GzScene3D.
    /// Disable them to work with the new MinimalScene plugin.
    public: bool legacy{true};

    /// \brief Flag for indicating whether we are spawning or not.
    public: bool isSpawning = false;

    /// \brief Flag for indicating whether the user is currently placing a
    /// resource with the shapes plugin or not
    public: bool isPlacing = false;

    /// \brief The SDF string of the resource to be used with plugins that spawn
    /// entities.
    public: std::string spawnSdfString;

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

    /// \brief The currently hovered mouse position in screen coordinates
    public: math::Vector2i mouseHoverPos = math::Vector2i::Zero;

    /// \brief Ray query for mouse clicks
    public: rendering::RayQueryPtr rayQuery{nullptr};

    /// \brief User camera
    public: rendering::CameraPtr camera;

    /// \brief Name of service for creating entity
    public: std::string createCmdService;

    /// \brief Name of the world
    public: std::string worldName;
  };
}

using namespace ignition;
using namespace gazebo;

// TODO(ahcorde): Replace this when this PR is on ign-rendering6
/////////////////////////////////////////////////
math::Vector3d LightsPrivate::ScreenToPlane(
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
void LightsPrivate::HandleModelPlacement()
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
  }
}

/////////////////////////////////////////////////
ignition::gazebo::Entity LightsPrivate::UniqueId()
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
void LightsPrivate::handleLights()
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

          igndbg << "Shapes plugin is using camera ["
                 << this->camera->Name() << "]" << std::endl;
          break;
        }
      }
    }
  }

  // Shapes
  IGN_PROFILE("IgnRenderer::Render Lights");
  if (this->isSpawning)
  {
    // Generate spawn preview
    rendering::VisualPtr rootVis = this->scene->RootVisual();
    sdf::Root root;
    if (!this->spawnSdfString.empty())
    {
      root.LoadSdfString(this->spawnSdfString);
    }
    else
    {
      ignwarn << "Failed to spawn: no SDF string or path" << std::endl;
    }
    this->isPlacing = this->GeneratePreview(root);
    this->isSpawning = false;
  }

  this->HandleModelPlacement();
}

/////////////////////////////////////////////////
void LightsPrivate::TerminateSpawnPreview()
{
  for (auto _id : this->previewIds)
    this->sceneManager.RemoveEntity(_id);
  this->previewIds.clear();
  this->isPlacing = false;
}

/////////////////////////////////////////////////
bool LightsPrivate::GeneratePreview(const sdf::Root &_sdf)
{
  // Terminate any pre-existing spawned entities
  this->TerminateSpawnPreview();

  if (nullptr == _sdf.Light())
  {
    ignwarn << "Only lights entities can be spawned at the moment." << std::endl;
    this->TerminateSpawnPreview();
    return false;
  }

  if (_sdf.Light())
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

/////////////////////////////////////////////////
Lights::Lights()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<LightsPrivate>())
{
}

/////////////////////////////////////////////////
Lights::~Lights() = default;

/////////////////////////////////////////////////
void Lights::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Lights";

  if (_pluginElem)
    {
      if (auto elem = _pluginElem->FirstChildElement("legacy"))
      {
        elem->QueryBoolText(&this->dataPtr->legacy);
      }
    }

    if (this->dataPtr->legacy)
    {
      igndbg << "Legacy mode is enabled; this plugin must be used with "
             << "GzScene3D." << std::endl;
    }
    else
    {
      igndbg << "Legacy mode is disabled; this plugin must be used with "
             << "MinimalScene." << std::endl;
    }

    // World name from window, to construct default topics and services
    auto worldNames = gui::worldNames();
    if (!worldNames.empty())
      this->dataPtr->worldName = worldNames[0].toStdString();

    ignition::gui::App()->findChild
      <ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void Lights::OnNewLightClicked(const QString &_sdfString)
{
  std::string modelSdfString = _sdfString.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  if (modelSdfString == "point")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.6\">"
                                 "<light type='point' name='pointlight'>"
                                   "<pose>0 0 2 0 0 0</pose>"
                                   "<cast_shadows>false</cast_shadows>"
                                   "<diffuse>0.5 0.5 0.5 1</diffuse>"
                                   "<specular>0.5 0.5 0.5 1</specular>"
                                   "<attenuation>"
                                     "<range>4</range>"
                                     "<constant>0.2</constant>"
                                     "<linear>0.5</linear>"
                                     "<quadratic>0.01</quadratic>"
                                   "</attenuation>"
                                 "</light>"
                                 "</sdf>");
  }
  else if (modelSdfString == "directional")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.6\">"
                                 "<light type='directional'"
                                  "name='directionallight'>"
                                   "<pose>0 0 2 0 0 0</pose>"
                                   "<cast_shadows>true</cast_shadows>"
                                   "<diffuse>0.8 0.8 0.8 1</diffuse>"
                                   "<specular>0.2 0.2 0.2 1</specular>"
                                   "<attenuation>"
                                     "<range>1000</range>"
                                     "<constant>0.9</constant>"
                                     "<linear>0.01</linear>"
                                     "<quadratic>0.001</quadratic>"
                                   "</attenuation>"
                                   "<direction>0 0 -1</direction>"
                                 "</light>"
                                 "</sdf>");
  }
  else if (modelSdfString == "spot")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.6\">"
                                 "<light type='spot' name='spotlight'>"
                                   "<pose>0 0 2 0 0 0</pose>"
                                   "<cast_shadows>true</cast_shadows>"
                                   "<diffuse>0.5 0.5 0.5 1</diffuse>"
                                   "<specular>0.5 0.5 0.5 1</specular>"
                                   "<attenuation>"
                                     "<range>4</range>"
                                     "<constant>0.2</constant>"
                                     "<linear>0.5</linear>"
                                     "<quadratic>0.01</quadratic>"
                                   "</attenuation>"
                                   "<direction>0 0 -1</direction>"
                                   "<spot>"
                                     "<inner_angle>0.1</inner_angle>"
                                     "<outer_angle>0.5</outer_angle>"
                                     "<falloff>0.8</falloff>"
                                   "</spot>"
                                 "</light>"
                                 "</sdf>");
  }
  else
  {
    ignwarn << "Invalid model string " << modelSdfString << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - point\n";
    ignwarn << " - directional\n";
    ignwarn << " - spot\n";
    return;
  }

  // Emit event to GzScene3D in legacy mode
  if (this->dataPtr->legacy)
  {
    ignition::gui::events::SpawnFromDescription event(modelSdfString);
    ignition::gui::App()->sendEvent(
          ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
          &event);
  }
  else
  {
    this->dataPtr->spawnSdfString = modelSdfString;
    this->dataPtr->isSpawning = true;
  }
}

/////////////////////////////////////////////////
bool Lights::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::Render::kType)
  {
    this->dataPtr->handleLights();
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
    this->dataPtr->mouseHoverPos = _e->Point();
    this->dataPtr->hoverDirty = true;
  }

  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Lights,
                    ignition::gui::Plugin)
